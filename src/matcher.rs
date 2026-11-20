//! Ridge-enhanced Normalized Cross-Correlation (NCC) matcher for 80x80 fingerprint frames.

use std::fs;
use std::io;
use std::path::Path;

pub const DEFAULT_MATCH_THRESHOLD: f32 = 0.45;
const TEMPLATE_MAGIC: &[u8; 5] = b"ELAN1";

/// Remove uneven pressure/illumination via local 9x9 mean subtraction,
/// returning a zero-mean `f32` ridge map.
pub fn preprocess_ridge_map(width: usize, height: usize, pixels: &[u8]) -> Vec<f32> {
    assert_eq!(pixels.len(), width * height);
    let radius = 4isize;
    let mut highpass = vec![0.0f32; width * height];

    for y in 0..height {
        for x in 0..width {
            let y0 = (y as isize - radius).max(0) as usize;
            let y1 = (y as isize + radius).min(height as isize - 1) as usize;
            let x0 = (x as isize - radius).max(0) as usize;
            let x1 = (x as isize + radius).min(width as isize - 1) as usize;

            let mut sum = 0u32;
            let mut count = 0u32;
            for yy in y0..=y1 {
                let row = yy * width;
                for xx in x0..=x1 {
                    sum += u32::from(pixels[row + xx]);
                    count += 1;
                }
            }
            let mean = (sum as f32) / (count as f32);
            highpass[y * width + x] = f32::from(pixels[y * width + x]) - mean;
        }
    }

    // Zero-mean over the full frame for clean NCC.
    let global_mean = highpass.iter().sum::<f32>() / (highpass.len() as f32);
    for v in &mut highpass {
        *v -= global_mean;
    }
    highpass
}

/// Compute Normalized Cross-Correlation between two preprocessed ridge maps
/// at translation `(dx, dy)` (probe shifted by `(dx, dy)` relative to template).
fn ncc_at_shift(
    width: usize,
    height: usize,
    tmpl: &[f32],
    probe: &[f32],
    dx: isize,
    dy: isize,
) -> f32 {
    let x_start = 0isize.max(-dx);
    let x_end = (width as isize).min(width as isize - dx);
    let y_start = 0isize.max(-dy);
    let y_end = (height as isize).min(height as isize - dy);

    if x_end <= x_start || y_end <= y_start {
        return 0.0;
    }

    let overlap = ((x_end - x_start) * (y_end - y_start)) as usize;
    // Require at least 45% overlap so extreme edge shifts cannot produce spurious peaks.
    if overlap * 100 < width * height * 45 {
        return 0.0;
    }

    let mut dot = 0.0f32;
    let mut norm_t = 0.0f32;
    let mut norm_p = 0.0f32;

    for y in y_start..y_end {
        let ty = (y as usize) * width;
        let py = ((y + dy) as usize) * width;
        for x in x_start..x_end {
            let t = tmpl[ty + (x as usize)];
            let p = probe[py + ((x + dx) as usize)];
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

/// Best translation-tolerant NCC score across `dx, dy in -16..=16`.
pub fn match_ridge_maps(width: usize, height: usize, tmpl: &[f32], probe: &[f32]) -> f32 {
    let max_shift = 16isize;
    let mut best = f32::NEG_INFINITY;
    let mut best_dx = 0isize;
    let mut best_dy = 0isize;

    // Coarse pass (step 2)
    for dy in (-max_shift..=max_shift).step_by(2) {
        for dx in (-max_shift..=max_shift).step_by(2) {
            let s = ncc_at_shift(width, height, tmpl, probe, dx, dy);
            if s > best {
                best = s;
                best_dx = dx;
                best_dy = dy;
            }
        }
    }

    // Fine pass (+/- 1 around coarse peak)
    for dy in (best_dy - 1)..=(best_dy + 1) {
        for dx in (best_dx - 1)..=(best_dx + 1) {
            let s = ncc_at_shift(width, height, tmpl, probe, dx, dy);
            if s > best {
                best = s;
            }
        }
    }

    best.max(0.0)
}

/// Compare a raw 8-bit probe frame against multiple enrolled 8-bit templates,
/// returning the highest similarity score in `0.0..=1.0`.
pub fn verify_against_templates(
    width: usize,
    height: usize,
    templates: &[Vec<u8>],
    probe: &[u8],
) -> f32 {
    let probe_map = preprocess_ridge_map(width, height, probe);
    templates
        .iter()
        .map(|t| {
            let tmpl_map = preprocess_ridge_map(width, height, t);
            match_ridge_maps(width, height, &tmpl_map, &probe_map)
        })
        .fold(0.0f32, f32::max)
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

    fn synthetic_ridge_frame(w: usize, h: usize, shift_x: isize) -> Vec<u8> {
        let mut out = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                let xx = (x as isize - shift_x) as f32;
                let yy = y as f32;
                let v = ((xx * 0.65 + yy * 0.15).sin() * 60.0 + 128.0).clamp(0.0, 255.0);
                out[y * w + x] = v as u8;
            }
        }
        out
    }

    #[test]
    fn matches_shifted_ridge_pattern() {
        let tmpl = synthetic_ridge_frame(80, 80, 0);
        let probe_same = synthetic_ridge_frame(80, 80, 5);
        let score = verify_against_templates(80, 80, &[tmpl], &probe_same);
        assert!(score > 0.85, "expected high score, got {score}");
    }

    #[test]
    fn rejects_orthogonal_pattern() {
        let tmpl = synthetic_ridge_frame(80, 80, 0);
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
    fn roundtrips_template_file() {
        let dir = std::env::temp_dir().join("elan_0c4b_test_tmpl");
        let path = dir.join("test.bin");
        let f1 = synthetic_ridge_frame(80, 80, 0);
        let f2 = synthetic_ridge_frame(80, 80, 2);
        save_templates(&path, 80, 80, &[f1.clone(), f2.clone()]).unwrap();
        let (w, h, loaded) = load_templates(&path).unwrap();
        assert_eq!((w, h), (80, 80));
        assert_eq!(loaded, vec![f1, f2]);
        let _ = fs::remove_dir_all(&dir);
    }
}
