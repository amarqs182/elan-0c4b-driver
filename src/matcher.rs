//! Fast ridge-enhanced Normalized Cross-Correlation (NCC) matcher for 80x80 fingerprint frames.
//!
//! Uses O(1)-per-pixel integral-image local contrast normalization, foreground variance masking,
//! a two-level spatial pyramid (`40x40` -> `80x80`), small-angle rotation search (`±12°`), and
//! early-exit across enrolled templates.

use std::fs;
use std::io;
use std::path::Path;

pub const DEFAULT_MATCH_THRESHOLD: f32 = 0.45;
pub const ROTATION_ANGLES_DEG: &[f32] = &[0.0, -6.0, 6.0, -12.0, 12.0];
pub const MIN_FOREGROUND_RATIO: f32 = 0.18;
pub const MIN_ADC_DIFF_SPAN: u16 = 60;

const LOCAL_STDDEV_FLOOR: f32 = 6.0;
const TEMPLATE_MAGIC: &[u8; 5] = b"ELAN1";

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchResult {
    pub score: f32,
    pub angle_deg: f32,
    pub dx: isize,
    pub dy: isize,
}

#[derive(Debug, Clone)]
pub struct PreparedTemplate {
    pub width: usize,
    pub height: usize,
    pub full: Vec<f32>,
    pub mask: Vec<u8>,
    pub half_w: usize,
    pub half_h: usize,
    pub half: Vec<f32>,
    pub foreground_ratio: f32,
}

impl PreparedTemplate {
    /// Preprocess an 8-bit frame in O(1) per pixel via summed-area tables (9x9 window),
    /// producing a locally variance-normalized ridge map, foreground mask, and 2x2 downsampled map.
    pub fn from_pixels(width: usize, height: usize, pixels: &[u8]) -> Self {
        assert_eq!(pixels.len(), width * height);
        let stride = width + 1;
        let mut sat = vec![0u32; (width + 1) * (height + 1)];
        let mut sat_sq = vec![0u64; (width + 1) * (height + 1)];

        for y in 0..height {
            let mut row_sum = 0u32;
            let mut row_sq = 0u64;
            let p_row = y * width;
            let prev_sat = y * stride;
            let cur_sat = (y + 1) * stride;
            for x in 0..width {
                let p = u32::from(pixels[p_row + x]);
                row_sum += p;
                row_sq += u64::from(p) * u64::from(p);
                sat[cur_sat + x + 1] = sat[prev_sat + x + 1] + row_sum;
                sat_sq[cur_sat + x + 1] = sat_sq[prev_sat + x + 1] + row_sq;
            }
        }

        let radius = 4isize;
        let n = width * height;
        let mut full = vec![0.0f32; n];
        let mut mask = vec![0u8; n];
        let mut active_sum = 0.0f32;
        let mut active_count = 0usize;

        for y in 0..height {
            let y0 = (y as isize - radius).max(0) as usize;
            let y1 = (y as isize + radius).min(height as isize - 1) as usize;
            for x in 0..width {
                let x0 = (x as isize - radius).max(0) as usize;
                let x1 = (x as isize + radius).min(width as isize - 1) as usize;

                let tl = y0 * stride + x0;
                let tr = y0 * stride + (x1 + 1);
                let bl = (y1 + 1) * stride + x0;
                let br = (y1 + 1) * stride + (x1 + 1);

                let sum = sat[br] + sat[tl] - sat[tr] - sat[bl];
                let sum_sq = sat_sq[br] + sat_sq[tl] - sat_sq[tr] - sat_sq[bl];
                let count = ((x1 - x0 + 1) * (y1 - y0 + 1)) as f32;

                let mean = (sum as f32) / count;
                let var = ((sum_sq as f32) / count) - mean * mean;
                let std_dev = var.max(0.0).sqrt();

                let idx = y * width + x;
                if std_dev >= LOCAL_STDDEV_FLOOR {
                    let norm = ((f32::from(pixels[idx]) - mean) / std_dev).clamp(-3.0, 3.0);
                    full[idx] = norm;
                    mask[idx] = 1;
                    active_sum += norm;
                    active_count += 1;
                }
            }
        }

        if active_count > 0 {
            let mean_active = active_sum / (active_count as f32);
            for (v, &m) in full.iter_mut().zip(mask.iter()) {
                if m != 0 {
                    *v -= mean_active;
                }
            }
        }

        let foreground_ratio = (active_count as f32) / (n.max(1) as f32);
        let (half_w, half_h, half) = downsample_2x2(width, height, &full);

        Self {
            width,
            height,
            full,
            mask,
            half_w,
            half_h,
            half,
            foreground_ratio,
        }
    }

    pub fn is_good_quality(&self) -> bool {
        self.foreground_ratio >= MIN_FOREGROUND_RATIO
    }
}

fn downsample_2x2(width: usize, height: usize, full: &[f32]) -> (usize, usize, Vec<f32>) {
    let half_w = width / 2;
    let half_h = height / 2;
    let mut half = vec![0.0f32; half_w * half_h];
    for hy in 0..half_h {
        let y0 = hy * 2;
        let y1 = y0 + 1;
        for hx in 0..half_w {
            let x0 = hx * 2;
            let x1 = x0 + 1;
            half[hy * half_w + hx] = 0.25
                * (full[y0 * width + x0]
                    + full[y0 * width + x1]
                    + full[y1 * width + x0]
                    + full[y1 * width + x1]);
        }
    }
    (half_w, half_h, half)
}

/// Remove uneven pressure/illumination via local 9x9 mean/variance normalization,
/// returning a zero-mean `f32` ridge map.
pub fn preprocess_ridge_map(width: usize, height: usize, pixels: &[u8]) -> Vec<f32> {
    PreparedTemplate::from_pixels(width, height, pixels).full
}

/// Rotate a preprocessed template around its center using bilinear interpolation.
pub fn rotate_prepared(tmpl: &PreparedTemplate, angle_deg: f32) -> PreparedTemplate {
    if angle_deg == 0.0 {
        return tmpl.clone();
    }

    let w = tmpl.width;
    let h = tmpl.height;
    let cx = (w as f32 - 1.0) * 0.5;
    let cy = (h as f32 - 1.0) * 0.5;
    let theta = angle_deg.to_radians();
    let cos_t = theta.cos();
    let sin_t = theta.sin();

    let mut full = vec![0.0f32; w * h];
    let mut mask = vec![0u8; w * h];
    let max_x = (w.saturating_sub(1)) as f32;
    let max_y = (h.saturating_sub(1)) as f32;

    for y in 0..h {
        let dy = y as f32 - cy;
        for x in 0..w {
            let dx = x as f32 - cx;
            let sx = cx + dx * cos_t + dy * sin_t;
            let sy = cy - dx * sin_t + dy * cos_t;

            if sx < 0.0 || sy < 0.0 || sx > max_x || sy > max_y {
                continue;
            }

            let x0 = sx.floor() as usize;
            let y0 = sy.floor() as usize;
            let x1 = (x0 + 1).min(w - 1);
            let y1 = (y0 + 1).min(h - 1);

            let i00 = y0 * w + x0;
            let i10 = y0 * w + x1;
            let i01 = y1 * w + x0;
            let i11 = y1 * w + x1;

            if tmpl.mask[i00] == 0 || tmpl.mask[i10] == 0 || tmpl.mask[i01] == 0 || tmpl.mask[i11] == 0 {
                continue;
            }

            let fx = sx - (x0 as f32);
            let fy = sy - (y0 as f32);
            let top = tmpl.full[i00] * (1.0 - fx) + tmpl.full[i10] * fx;
            let bot = tmpl.full[i01] * (1.0 - fx) + tmpl.full[i11] * fx;

            let idx = y * w + x;
            full[idx] = top * (1.0 - fy) + bot * fy;
            mask[idx] = 1;
        }
    }

    let (half_w, half_h, half) = downsample_2x2(w, h, &full);
    PreparedTemplate {
        width: w,
        height: h,
        full,
        mask,
        half_w,
        half_h,
        half,
        foreground_ratio: tmpl.foreground_ratio,
    }
}

fn ncc_half(hw: usize, hh: usize, tmpl_half: &[f32], probe_half: &[f32], hdx: isize, hdy: isize) -> f32 {
    let x_start = 0isize.max(-hdx);
    let x_end = (hw as isize).min(hw as isize - hdx);
    let y_start = 0isize.max(-hdy);
    let y_end = (hh as isize).min(hh as isize - hdy);

    if x_end <= x_start || y_end <= y_start {
        return 0.0;
    }

    let overlap = ((x_end - x_start) * (y_end - y_start)) as usize;
    if overlap * 100 < hw * hh * 40 {
        return 0.0;
    }

    let mut dot = 0.0f32;
    let mut norm_t = 0.0f32;
    let mut norm_p = 0.0f32;

    for y in y_start..y_end {
        let ty = (y as usize) * hw;
        let py = ((y + hdy) as usize) * hw;
        for x in x_start..x_end {
            let t = tmpl_half[ty + (x as usize)];
            let p = probe_half[py + ((x + hdx) as usize)];
            dot += t * p;
            norm_t += t * t;
            norm_p += p * p;
        }
    }

    let denom = (norm_t * norm_p).sqrt();
    if denom <= 1e-6 {
        0.0
    } else {
        dot / denom
    }
}

#[allow(clippy::too_many_arguments)]
fn ncc_masked(
    w: usize,
    h: usize,
    tmpl: &[f32],
    t_mask: &[u8],
    probe: &[f32],
    p_mask: &[u8],
    dx: isize,
    dy: isize,
    min_active_overlap: usize,
) -> f32 {
    let x_start = 0isize.max(-dx);
    let x_end = (w as isize).min(w as isize - dx);
    let y_start = 0isize.max(-dy);
    let y_end = (h as isize).min(h as isize - dy);

    if x_end <= x_start || y_end <= y_start {
        return 0.0;
    }

    let geo_overlap = ((x_end - x_start) * (y_end - y_start)) as usize;
    if geo_overlap < min_active_overlap {
        return 0.0;
    }

    let mut dot = 0.0f32;
    let mut norm_t = 0.0f32;
    let mut norm_p = 0.0f32;
    let mut active_overlap = 0usize;

    for y in y_start..y_end {
        let ty = (y as usize) * w;
        let py = ((y + dy) as usize) * w;
        for x in x_start..x_end {
            let ti = ty + (x as usize);
            let pi = py + ((x + dx) as usize);
            if t_mask[ti] != 0 && p_mask[pi] != 0 {
                let t = tmpl[ti];
                let p = probe[pi];
                dot += t * p;
                norm_t += t * t;
                norm_p += p * p;
                active_overlap += 1;
            }
        }
    }

    if active_overlap < min_active_overlap {
        return 0.0;
    }

    let denom = (norm_t * norm_p).sqrt();
    if denom <= 1e-6 {
        0.0
    } else {
        (dot / denom).max(0.0)
    }
}

/// Two-level pyramid NCC search (`40x40` top-3 coarse peaks -> `80x80` masked fine search).
pub fn match_pair_pyramid(
    tmpl: &PreparedTemplate,
    probe: &PreparedTemplate,
    angle_deg: f32,
) -> MatchResult {
    let mut coarse = Vec::with_capacity(17 * 17);
    for hdy in -8isize..=8 {
        for hdx in -8isize..=8 {
            let s = ncc_half(tmpl.half_w, tmpl.half_h, &tmpl.half, &probe.half, hdx, hdy);
            coarse.push((s, hdx, hdy));
        }
    }
    coarse.sort_by(|a, b| b.0.total_cmp(&a.0));

    let min_active_overlap = (tmpl.width * tmpl.height) / 7;
    let mut visited = [false; 33 * 33];
    let mut best = MatchResult {
        score: 0.0,
        angle_deg,
        dx: 0,
        dy: 0,
    };

    for &(_, hdx, hdy) in coarse.iter().take(3) {
        let dx_lo = (2 * hdx - 2).max(-16);
        let dx_hi = (2 * hdx + 2).min(16);
        let dy_lo = (2 * hdy - 2).max(-16);
        let dy_hi = (2 * hdy + 2).min(16);

        for dy in dy_lo..=dy_hi {
            for dx in dx_lo..=dx_hi {
                let v_idx = ((dy + 16) as usize) * 33 + ((dx + 16) as usize);
                if visited[v_idx] {
                    continue;
                }
                visited[v_idx] = true;

                let s = ncc_masked(
                    tmpl.width,
                    tmpl.height,
                    &tmpl.full,
                    &tmpl.mask,
                    &probe.full,
                    &probe.mask,
                    dx,
                    dy,
                    min_active_overlap,
                );
                if s > best.score {
                    best = MatchResult {
                        score: s,
                        angle_deg,
                        dx,
                        dy,
                    };
                }
            }
        }
    }

    best
}

/// Best translation-tolerant NCC score across `dx, dy in -16..=16` at 0° rotation.
pub fn match_ridge_maps(width: usize, height: usize, tmpl: &[f32], probe: &[f32]) -> f32 {
    let tmpl_p = PreparedTemplate {
        width,
        height,
        full: tmpl.to_vec(),
        mask: vec![1u8; width * height],
        half_w: width / 2,
        half_h: height / 2,
        half: downsample_2x2(width, height, tmpl).2,
        foreground_ratio: 1.0,
    };
    let probe_p = PreparedTemplate {
        width,
        height,
        full: probe.to_vec(),
        mask: vec![1u8; width * height],
        half_w: width / 2,
        half_h: height / 2,
        half: downsample_2x2(width, height, probe).2,
        foreground_ratio: 1.0,
    };
    match_pair_pyramid(&tmpl_p, &probe_p, 0.0).score
}

/// Compare a preprocessed probe against cached `PreparedTemplate`s across `ROTATION_ANGLES_DEG`.
/// When `early_exit_threshold` is `Some(thr)`, stops and returns as soon as `best.score >= thr`.
pub fn verify_prepared(
    templates: &[PreparedTemplate],
    probe: &PreparedTemplate,
    early_exit_threshold: Option<f32>,
) -> MatchResult {
    let mut best = MatchResult {
        score: 0.0,
        angle_deg: 0.0,
        dx: 0,
        dy: 0,
    };

    for &angle in ROTATION_ANGLES_DEG {
        let rotated;
        let probe_ref = if angle == 0.0 {
            probe
        } else {
            rotated = rotate_prepared(probe, angle);
            &rotated
        };

        for tmpl in templates {
            if tmpl.width != probe_ref.width || tmpl.height != probe_ref.height {
                continue;
            }
            let res = match_pair_pyramid(tmpl, probe_ref, angle);
            if res.score > best.score {
                best = res;
                if let Some(thr) = early_exit_threshold {
                    if best.score >= thr {
                        return best;
                    }
                }
            }
        }
    }

    best
}

/// Preprocess raw frames and return the full `MatchResult` (score, angle, shift).
pub fn verify_with_details(
    width: usize,
    height: usize,
    templates: &[Vec<u8>],
    probe: &[u8],
) -> MatchResult {
    let prepared: Vec<PreparedTemplate> = templates
        .iter()
        .map(|t| PreparedTemplate::from_pixels(width, height, t))
        .collect();
    let probe_p = PreparedTemplate::from_pixels(width, height, probe);
    verify_prepared(&prepared, &probe_p, None)
}

/// Compare a raw 8-bit probe frame against multiple enrolled 8-bit templates,
/// returning the highest similarity score in `0.0..=1.0`.
pub fn verify_against_templates(
    width: usize,
    height: usize,
    templates: &[Vec<u8>],
    probe: &[u8],
) -> f32 {
    verify_with_details(width, height, templates, probe).score
}

/// Save enrolled 8-bit frames to a binary template file.
pub fn save_templates(
    path: &Path,
    width: u16,
    height: u16,
    templates: &[Vec<u8>],
) -> io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let expected = usize::from(width) * usize::from(height);
    let mut buf = Vec::with_capacity(11 + templates.len() * expected);
    buf.extend_from_slice(TEMPLATE_MAGIC);
    buf.extend_from_slice(&width.to_le_bytes());
    buf.extend_from_slice(&height.to_le_bytes());
    buf.extend_from_slice(&(templates.len() as u16).to_le_bytes());
    for t in templates {
        if t.len() != expected {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "template frame size mismatch",
            ));
        }
        buf.extend_from_slice(t);
    }
    fs::write(path, buf)
}

/// Load enrolled 8-bit frames from a binary template file.
pub fn load_templates(path: &Path) -> io::Result<(u16, u16, Vec<Vec<u8>>)> {
    let data = fs::read(path)?;
    if data.len() < 11 || &data[0..5] != TEMPLATE_MAGIC {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid template header",
        ));
    }
    let width = u16::from_le_bytes([data[5], data[6]]);
    let height = u16::from_le_bytes([data[7], data[8]]);
    let count = u16::from_le_bytes([data[9], data[10]]) as usize;
    let frame_len = usize::from(width) * usize::from(height);
    if data.len() != 11 + count * frame_len {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "truncated template file",
        ));
    }
    let mut frames = Vec::with_capacity(count);
    for i in 0..count {
        let start = 11 + i * frame_len;
        frames.push(data[start..start + frame_len].to_vec());
    }
    Ok((width, height, frames))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_ridge_frame(w: usize, h: usize, shift_x: isize, angle_deg: f32) -> Vec<u8> {
        let mut out = vec![0u8; w * h];
        let cx = (w as f32 - 1.0) * 0.5;
        let cy = (h as f32 - 1.0) * 0.5;
        let theta = angle_deg.to_radians();
        let cos_t = theta.cos();
        let sin_t = theta.sin();

        for y in 0..h {
            for x in 0..w {
                let dx = (x as isize - shift_x) as f32 - cx;
                let dy = y as f32 - cy;
                let rx = cx + dx * cos_t - dy * sin_t;
                let ry = cy + dx * sin_t + dy * cos_t;
                let v = ((rx * 0.65 + ry * 0.15).sin() * 60.0 + 128.0).clamp(0.0, 255.0);
                out[y * w + x] = v as u8;
            }
        }
        out
    }

    #[test]
    fn matches_shifted_ridge_pattern() {
        let tmpl = synthetic_ridge_frame(80, 80, 0, 0.0);
        let probe_same = synthetic_ridge_frame(80, 80, 5, 0.0);
        let score = verify_against_templates(80, 80, &[tmpl], &probe_same);
        assert!(score > 0.85, "expected high score, got {score}");
    }

    #[test]
    fn matches_rotated_ridge_pattern() {
        let tmpl = synthetic_ridge_frame(80, 80, 0, 0.0);
        let probe_rot = synthetic_ridge_frame(80, 80, 2, 10.0);
        let res = verify_with_details(80, 80, &[tmpl], &probe_rot);
        assert!(
            res.score > 0.70,
            "expected high rotated score, got {} at {}deg",
            res.score,
            res.angle_deg
        );
    }

    #[test]
    fn rejects_orthogonal_pattern() {
        let tmpl = synthetic_ridge_frame(80, 80, 0, 0.0);
        let mut ortho = vec![0u8; 80 * 80];
        for y in 0..80 {
            for x in 0..80 {
                let v = ((-(x as f32) * 0.15 + (y as f32) * 0.65).sin() * 60.0 + 128.0)
                    .clamp(0.0, 255.0);
                ortho[y * 80 + x] = v as u8;
            }
        }
        let score = verify_against_templates(80, 80, &[tmpl], &ortho);
        assert!(score < 0.25, "expected low score, got {score}");
    }

    #[test]
    fn rejects_low_quality_blank_frame() {
        let flat = vec![120u8; 80 * 80];
        let prep = PreparedTemplate::from_pixels(80, 80, &flat);
        assert!(
            !prep.is_good_quality(),
            "expected flat frame to fail quality check, got fg={}",
            prep.foreground_ratio
        );
    }

    #[test]
    fn early_exit_returns_matching_result() {
        let t1 = PreparedTemplate::from_pixels(80, 80, &synthetic_ridge_frame(80, 80, 0, 0.0));
        let t2 = PreparedTemplate::from_pixels(80, 80, &synthetic_ridge_frame(80, 80, 10, 0.0));
        let probe = PreparedTemplate::from_pixels(80, 80, &synthetic_ridge_frame(80, 80, 4, 0.0));
        let res = verify_prepared(&[t1, t2], &probe, Some(DEFAULT_MATCH_THRESHOLD));
        assert!(res.score >= DEFAULT_MATCH_THRESHOLD);
        assert_eq!(res.angle_deg, 0.0);
    }

    #[test]
    fn roundtrips_template_file() {
        let dir = std::env::temp_dir().join("elan_0c4b_test_tmpl");
        let path = dir.join("test.bin");
        let f1 = synthetic_ridge_frame(80, 80, 0, 0.0);
        let f2 = synthetic_ridge_frame(80, 80, 2, 0.0);
        save_templates(&path, 80, 80, &[f1.clone(), f2.clone()]).unwrap();
        let (w, h, loaded) = load_templates(&path).unwrap();
        assert_eq!((w, h), (80, 80));
        assert_eq!(loaded, vec![f1, f2]);
        let _ = fs::remove_dir_all(&dir);
    }
}
