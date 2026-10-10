//! Splitting an H.264 Annex B byte stream into access units, for the
//! command-line tools. (In a pipeline, `Membrane.H264.Parser` does this.)
//!
//! A new access unit starts (H.264 7.4.1.2.3) at an access unit delimiter,
//! SPS, PPS, SEI or NAL types 14–18 after a picture's slices, or at a slice
//! whose `first_mb_in_slice` is 0 after a picture's slices. That last test
//! assumes slices arrive in order, which holds for everything but arbitrary
//! slice order streams (Baseline only, and rare).

/// The NAL units in `data`: (start of the start code, start of the NAL
/// header, end).
fn nal_units(data: &[u8]) -> Vec<(usize, usize, usize)> {
    let mut starts = Vec::new();
    let mut i = 0;
    while i + 3 <= data.len() {
        if data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1 {
            let code_start = if i > 0 && data[i - 1] == 0 { i - 1 } else { i };
            starts.push((code_start, i + 3));
            i += 3;
        } else {
            i += 1;
        }
    }

    let mut nals = Vec::with_capacity(starts.len());
    for (n, &(code_start, header)) in starts.iter().enumerate() {
        let end = starts.get(n + 1).map_or(data.len(), |&(next, _)| next);
        if header < end {
            nals.push((code_start, header, end));
        }
    }
    nals
}

/// Access units in `data`, each with its start codes.
pub fn access_units(data: &[u8]) -> Vec<&[u8]> {
    let mut units = Vec::new();
    let mut au_start: Option<usize> = None;
    let mut seen_vcl = false;

    for (code_start, header, end) in nal_units(data) {
        let nal_type = data[header] & 0x1f;
        let vcl = nal_type == 1 || nal_type == 5;
        let first_mb_zero = vcl && header + 1 < end && data[header + 1] & 0x80 != 0;
        let starts_au = matches!(nal_type, 6..=9 | 14..=18) || first_mb_zero;

        if seen_vcl && starts_au {
            if let Some(start) = au_start {
                units.push(&data[start..code_start]);
            }
            au_start = Some(code_start);
            seen_vcl = false;
        }

        au_start.get_or_insert(code_start);
        seen_vcl |= vcl;
    }

    if let Some(start) = au_start {
        units.push(&data[start..]);
    }
    units
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_on_new_pictures_and_parameter_sets() {
        let sps = [0, 0, 0, 1, 0x67, 1];
        let pps = [0, 0, 0, 1, 0x68, 2];
        let idr = [0, 0, 1, 0x65, 0x88, 3];
        let p1 = [0, 0, 1, 0x41, 0x9a, 4];
        let p1_slice2 = [0, 0, 1, 0x41, 0x40, 5];
        let p2 = [0, 0, 1, 0x41, 0x9a, 6];
        let stream: Vec<u8> = [&sps[..], &pps, &idr, &p1, &p1_slice2, &p2].concat();

        let aus = access_units(&stream);
        assert_eq!(aus.len(), 3);
        assert_eq!(aus[0], [&sps[..], &pps, &idr].concat());
        assert_eq!(aus[1], [&p1[..], &p1_slice2].concat());
        assert_eq!(aus[2], p2);
    }
}
