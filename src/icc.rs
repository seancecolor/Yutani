use lcms2::{Profile, ToneCurve};

pub fn generate_bt1886_profile(white_lum: f64, black_lum: f64) {
    let gamma = 2.4;
    let lw_pow = white_lum.powf(1.0 / gamma);
    let lb_pow = black_lum.powf(1.0 / gamma);
    
    let a = (lw_pow - lb_pow).powf(gamma);
    let b = lb_pow / (lw_pow - lb_pow);

    let mut curve_data = Vec::with_capacity(1024);
    for i in 0..1024 {
        let v = i as f64 / 1023.0;
        let v_plus_b = (v + b).max(0.0);
        let l = a * v_plus_b.powf(gamma);
        
        let l_norm = (l - black_lum) / (white_lum - black_lum);
        curve_data.push(l_norm as f32);
    }

    let bt1886_curve = ToneCurve::new_tabulated(&curve_data);
    let curves = [&bt1886_curve, &bt1886_curve, &bt1886_curve];

    // BT.709 primaries used as baseline (you would measure the actual PA32UCDM primaries here)
    let mut profile = Profile::new_rgb(
        &lcms2::CIExyY { x: 0.3127, y: 0.3290, Y: 1.0 }, 
        &lcms2::CIExyYTRIPLE {
            Red: lcms2::CIExyY { x: 0.64, y: 0.33, Y: 0.2126 },
            Green: lcms2::CIExyY { x: 0.30, y: 0.60, Y: 0.7152 },
            Blue: lcms2::CIExyY { x: 0.15, y: 0.06, Y: 0.0722 },
        },
        &curves,
    ).unwrap();

    profile.save_file("Yutani_PA32UCDM_BT1886.icc").unwrap();
    println!("Saved Yutani_PA32UCDM_BT1886.icc");
}
