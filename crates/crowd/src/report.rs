//! 区画ごとの集計（通過人数・平均待ち時間・平均滞留時間・最大密度）と CSV / PNG 出力。
//!
//! - 区画: 範囲を `size` [m] 四方に区切ったマス（範囲の南西端から）
//! - 通過人数: その区画に入った延べ人数
//! - 平均滞留時間: 1 回の通過あたりに区画内にいた時間
//! - 平均待ち時間: 1 回の通過あたりに速度 `wait_speed` [m/s] 未満で止まっていた時間（信号・混雑待ち）
//! - 最大人数・最大密度: 同時に区画内にいた人数の最大と、歩ける面積あたりの密度 [人/m²]

use crate::{png, Simulation};
use glam::Vec2;
use std::io::Write;
use walkmap::WalkGrid;

const NONE: u32 = u32::MAX;

#[derive(Debug, Clone)]
pub struct TimelineRow {
    pub t: f32,
    /// 区間内の平均歩行者数
    pub walking: f32,
    pub mean_speed: f32,
    /// 止まっている人の割合
    pub stopped_ratio: f32,
    pub cars: f32,
}

#[derive(Debug, Clone)]
pub struct ZoneStats {
    pub size: f32,
    pub nx: usize,
    pub ny: usize,
    /// 区画内の歩ける面積 [m²]
    pub walk_area: Vec<f32>,
    pub ped_passes: Vec<u32>,
    pub dwell: Vec<f64>,
    pub wait: Vec<f64>,
    pub peak: Vec<u32>,
    pub car_passes: Vec<u32>,
    /// グリッドのセルごとの延べ滞在（人・ステップ）
    pub heat: Vec<u32>,
    pub elapsed: f32,
    pub wait_speed: f32,
    pub timeline: Vec<TimelineRow>,
    pub timeline_every: f32,
    cell_zone: Vec<u32>,
    last_ped: Vec<u32>,
    last_car: Vec<u32>,
    occ: Vec<u32>,
    tl_acc: [f64; 5],
}

impl ZoneStats {
    pub fn new(grid: &WalkGrid, size: f32) -> Self {
        let size = size.max(grid.cell);
        let nx = ((grid.w as f32 * grid.cell / size).ceil() as usize).max(1);
        let ny = ((grid.h as f32 * grid.cell / size).ceil() as usize).max(1);
        let mut walk_area = vec![0.0; nx * ny];
        let mut cell_zone = vec![0; grid.w * grid.h];
        for y in 0..grid.h {
            for x in 0..grid.w {
                let p = grid.center(x, y) - grid.origin;
                let z = ((p.y / size) as usize).min(ny - 1) * nx + ((p.x / size) as usize).min(nx - 1);
                let i = grid.idx(x, y);
                cell_zone[i] = z as u32;
                if grid.walkable[i] {
                    walk_area[z] += grid.cell * grid.cell;
                }
            }
        }
        Self {
            size,
            nx,
            ny,
            walk_area,
            ped_passes: vec![0; nx * ny],
            dwell: vec![0.0; nx * ny],
            wait: vec![0.0; nx * ny],
            peak: vec![0; nx * ny],
            car_passes: vec![0; nx * ny],
            heat: vec![0; grid.w * grid.h],
            elapsed: 0.0,
            wait_speed: 0.3,
            timeline: vec![],
            timeline_every: 60.0,
            cell_zone,
            last_ped: vec![],
            last_car: vec![],
            occ: vec![0; nx * ny],
            tl_acc: [0.0; 5],
        }
    }

    /// 集計をゼロに戻す（区画の区切りはそのまま）。
    pub fn reset(&mut self) {
        self.ped_passes.iter_mut().for_each(|v| *v = 0);
        self.car_passes.iter_mut().for_each(|v| *v = 0);
        self.peak.iter_mut().for_each(|v| *v = 0);
        self.dwell.iter_mut().for_each(|v| *v = 0.0);
        self.wait.iter_mut().for_each(|v| *v = 0.0);
        self.heat.iter_mut().for_each(|v| *v = 0);
        self.elapsed = 0.0;
        self.timeline.clear();
        self.last_ped.clear();
        self.last_car.clear();
        self.tl_acc = [0.0; 5];
    }

    fn zone_of(&self, g: &WalkGrid, p: Vec2) -> Option<usize> {
        g.cell_of(p).map(|(x, y)| self.cell_zone[g.idx(x, y)] as usize)
    }

    /// 1 ステップ（dt 秒）分を集計する。`cars` は (位置, 走行中か)。
    pub fn record(&mut self, sim: &Simulation, cars: impl Iterator<Item = (Vec2, bool)>, dt: f32) {
        let g = &sim.grid;
        if self.last_ped.len() < sim.agents.len() {
            self.last_ped.resize(sim.agents.len(), NONE);
        }
        self.occ.iter_mut().for_each(|o| *o = 0);
        let (mut n, mut vsum, mut stopped) = (0usize, 0.0f64, 0usize);
        for (i, a) in sim.agents.iter().enumerate() {
            if a.arrived {
                self.last_ped[i] = NONE;
                continue;
            }
            let Some((cx, cy)) = g.cell_of(a.pos) else {
                self.last_ped[i] = NONE;
                continue;
            };
            let ci = g.idx(cx, cy);
            self.heat[ci] += 1;
            let z = self.cell_zone[ci];
            if self.last_ped[i] != z {
                self.ped_passes[z as usize] += 1;
                self.last_ped[i] = z;
            }
            let zi = z as usize;
            self.dwell[zi] += dt as f64;
            let v = a.vel.length();
            if v < self.wait_speed {
                self.wait[zi] += dt as f64;
                stopped += 1;
            }
            self.occ[zi] += 1;
            n += 1;
            vsum += v as f64;
        }
        for (p, o) in self.peak.iter_mut().zip(&self.occ) {
            *p = (*p).max(*o);
        }
        let mut ncar = 0usize;
        for (i, (p, active)) in cars.enumerate() {
            if self.last_car.len() <= i {
                self.last_car.resize(i + 1, NONE);
            }
            let z = if active { self.zone_of(g, p) } else { None };
            match z {
                Some(z) => {
                    ncar += 1;
                    if self.last_car[i] != z as u32 {
                        self.car_passes[z] += 1;
                        self.last_car[i] = z as u32;
                    }
                }
                None => self.last_car[i] = NONE,
            }
        }
        // 時系列（timeline_every 秒ごとの平均）
        let w = dt as f64;
        self.tl_acc[0] += n as f64 * w;
        self.tl_acc[1] += vsum * w;
        self.tl_acc[2] += stopped as f64 * w;
        self.tl_acc[3] += ncar as f64 * w;
        self.tl_acc[4] += w;
        self.elapsed += dt;
        let bucket_end = (self.timeline.len() + 1) as f32 * self.timeline_every;
        if self.elapsed + dt * 0.5 >= bucket_end {
            self.flush_timeline(bucket_end);
        }
    }

    fn flush_timeline(&mut self, t: f32) {
        let [n, v, s, c, w] = self.tl_acc;
        if w <= 0.0 {
            return;
        }
        self.timeline.push(TimelineRow {
            t,
            walking: (n / w) as f32,
            mean_speed: if n > 0.0 { (v / n) as f32 } else { 0.0 },
            stopped_ratio: if n > 0.0 { (s / n) as f32 } else { 0.0 },
            cars: (c / w) as f32,
        });
        self.tl_acc = [0.0; 5];
    }

    /// 1 時間あたりの通過人数
    pub fn passes_per_h(&self, z: usize) -> f32 {
        self.ped_passes[z] as f32 * 3600.0 / self.elapsed.max(1e-3)
    }
    pub fn cars_per_h(&self, z: usize) -> f32 {
        self.car_passes[z] as f32 * 3600.0 / self.elapsed.max(1e-3)
    }
    pub fn avg_dwell(&self, z: usize) -> f32 {
        (self.dwell[z] / self.ped_passes[z].max(1) as f64) as f32
    }
    pub fn avg_wait(&self, z: usize) -> f32 {
        (self.wait[z] / self.ped_passes[z].max(1) as f64) as f32
    }
    pub fn peak_density(&self, z: usize) -> f32 {
        if self.walk_area[z] > 0.0 {
            self.peak[z] as f32 / self.walk_area[z]
        } else {
            0.0
        }
    }
    /// 1 秒あたりのセル滞在（比較用に時間で正規化）
    pub fn heat_rate(&self) -> Vec<f32> {
        let k = 1.0 / self.elapsed.max(1e-3);
        self.heat.iter().map(|&h| h as f32 * k).collect()
    }
    pub fn zone_center(&self, g: &WalkGrid, z: usize) -> Vec2 {
        g.origin + Vec2::new((z % self.nx) as f32 + 0.5, (z / self.nx) as f32 + 0.5) * self.size
    }

    /// 区画ごとの CSV。`before` があれば比較列（Before と差分、どちらも 1 時間あたりに正規化）を足す。
    pub fn write_zones_csv(
        &self,
        g: &WalkGrid,
        before: Option<&ZoneStats>,
        out: &mut impl Write,
    ) -> std::io::Result<()> {
        write!(
            out,
            "zone,ix,iy,lat,lon,walkable_m2,ped_passes,ped_passes_per_h,avg_dwell_s,avg_wait_s,peak_people,peak_density_per_m2,car_passes,car_passes_per_h"
        )?;
        let before = before.filter(|b| b.nx == self.nx && b.ny == self.ny);
        if before.is_some() {
            write!(
                out,
                ",before_ped_passes_per_h,diff_ped_passes_per_h,before_avg_wait_s,diff_avg_wait_s,before_avg_dwell_s,diff_avg_dwell_s,before_peak_density_per_m2,diff_peak_density_per_m2,before_car_passes_per_h,diff_car_passes_per_h"
            )?;
        }
        writeln!(out)?;
        for z in 0..self.nx * self.ny {
            if self.walk_area[z] <= 0.0 {
                continue;
            }
            let (lat, lon) = g.projection.to_geo(self.zone_center(g, z));
            write!(
                out,
                "{z},{},{},{lat:.7},{lon:.7},{:.1},{},{:.1},{:.2},{:.2},{},{:.3},{},{:.1}",
                z % self.nx,
                z / self.nx,
                self.walk_area[z],
                self.ped_passes[z],
                self.passes_per_h(z),
                self.avg_dwell(z),
                self.avg_wait(z),
                self.peak[z],
                self.peak_density(z),
                self.car_passes[z],
                self.cars_per_h(z),
            )?;
            if let Some(b) = before {
                write!(
                    out,
                    ",{:.1},{:.1},{:.2},{:.2},{:.2},{:.2},{:.3},{:.3},{:.1},{:.1}",
                    b.passes_per_h(z),
                    self.passes_per_h(z) - b.passes_per_h(z),
                    b.avg_wait(z),
                    self.avg_wait(z) - b.avg_wait(z),
                    b.avg_dwell(z),
                    self.avg_dwell(z) - b.avg_dwell(z),
                    b.peak_density(z),
                    self.peak_density(z) - b.peak_density(z),
                    b.cars_per_h(z),
                    self.cars_per_h(z) - b.cars_per_h(z),
                )?;
            }
            writeln!(out)?;
        }
        Ok(())
    }

    /// 時系列の CSV。`clock` で時刻列を作る（開始時刻つきのシナリオなら "17:05:00" など）。
    pub fn write_timeline_csv(
        &self,
        clock: impl Fn(f32) -> String,
        out: &mut impl Write,
    ) -> std::io::Result<()> {
        writeln!(out, "t_end_s,clock,walking_avg,mean_speed_mps,stopped_ratio,cars_avg")?;
        let mut rows = self.timeline.clone();
        let [n, v, s, c, w] = self.tl_acc;
        if w > 0.0 {
            rows.push(TimelineRow {
                t: self.elapsed,
                walking: (n / w) as f32,
                mean_speed: if n > 0.0 { (v / n) as f32 } else { 0.0 },
                stopped_ratio: if n > 0.0 { (s / n) as f32 } else { 0.0 },
                cars: (c / w) as f32,
            });
        }
        for r in rows {
            writeln!(
                out,
                "{:.0},{},{:.1},{:.3},{:.3},{:.1}",
                r.t,
                clock(r.t),
                r.walking,
                r.mean_speed,
                r.stopped_ratio,
                r.cars
            )?;
        }
        Ok(())
    }

    /// 区画の値を色分けした地図 PNG（北が上、下端に凡例）。
    pub fn zone_png(&self, g: &WalkGrid, values: &[f32]) -> Vec<u8> {
        let max = values.iter().copied().fold(0.0, f32::max);
        let mut img = Canvas::base(g);
        for y in 0..g.h {
            for x in 0..g.w {
                let i = g.idx(x, y);
                if !g.walkable[i] {
                    continue;
                }
                let z = self.cell_zone[i] as usize;
                let v = values[z];
                let px = (x as f32 * g.cell / self.size).fract() * self.size;
                let py = (y as f32 * g.cell / self.size).fract() * self.size;
                let border = px < g.cell || py < g.cell;
                let c = if v <= 0.0 || max <= 0.0 {
                    [205, 205, 200]
                } else {
                    seq_color(v / max)
                };
                img.set(x, y, if border { shade(c, 0.85) } else { c });
            }
        }
        img.legend(0.0, max, false);
        img.encode()
    }
}

/// セル単位のヒートマップ PNG（北が上、灰=歩行可能, 黒=建物, 黄〜赤=通行量、下端に凡例）。
pub fn heat_png(g: &WalkGrid, heat: &[f32]) -> Vec<u8> {
    let max = heat.iter().copied().fold(0.0, f32::max);
    let mut img = Canvas::base(g);
    for y in 0..g.h {
        for x in 0..g.w {
            let v = heat[g.idx(x, y)];
            if v > 0.0 && max > 0.0 {
                img.set(x, y, seq_color(v.ln_1p() / max.ln_1p()));
            }
        }
    }
    img.legend(0.0, max, false);
    img.encode()
}

/// Before/After の差分 PNG。`after - before`（どちらも時間で正規化した値）を赤（増）・青（減）で塗る。
/// `closed` が true のセル（After で通行止め）は斜線で示す。
pub fn diff_png(g: &WalkGrid, before: &[f32], after: &[f32], closed: &[bool]) -> Vec<u8> {
    // 1 セルの差は細い線になって見えにくいので、歩ける範囲で半径 2 セルにならす
    let (before, after) = (blur(g, before, 2), blur(g, after, 2));
    let diff: Vec<f32> = after.iter().zip(&before).map(|(a, b)| a - b).collect();
    // 外れ値に引きずられないよう 99 パーセンタイルで正規化
    let mut abs: Vec<f32> = diff.iter().map(|d| d.abs()).filter(|d| *d > 0.0).collect();
    abs.sort_by(|a, b| a.total_cmp(b));
    let scale = abs.get(abs.len() * 99 / 100).copied().unwrap_or(1.0).max(1e-6);
    let mut img = Canvas::base(g);
    for y in 0..g.h {
        for x in 0..g.w {
            let i = g.idx(x, y);
            if closed.get(i).copied().unwrap_or(false) {
                img.set(x, y, if (x + y) % 6 < 3 { [40, 40, 40] } else { [250, 210, 0] });
                continue;
            }
            if !g.walkable[i] || diff[i] == 0.0 {
                continue;
            }
            img.set(x, y, div_color((diff[i] / scale).clamp(-1.0, 1.0)));
        }
    }
    img.legend(-scale, scale, true);
    img.encode()
}

/// 歩行可能セルだけでの箱型平均
fn blur(g: &WalkGrid, f: &[f32], r: i64) -> Vec<f32> {
    let mut out = vec![0.0; f.len()];
    for y in 0..g.h as i64 {
        for x in 0..g.w as i64 {
            let i = g.idx(x as usize, y as usize);
            if !g.walkable[i] {
                continue;
            }
            let (mut s, mut n) = (0.0, 0.0);
            for dy in -r..=r {
                for dx in -r..=r {
                    let (nx, ny) = (x + dx, y + dy);
                    if nx < 0 || ny < 0 || nx >= g.w as i64 || ny >= g.h as i64 {
                        continue;
                    }
                    let j = g.idx(nx as usize, ny as usize);
                    if g.walkable[j] {
                        s += f[j];
                        n += 1.0;
                    }
                }
            }
            out[i] = s / n;
        }
    }
    out
}

fn seq_color(t: f32) -> [u8; 3] {
    let t = t.clamp(0.0, 1.0);
    [255, (230.0 * (1.0 - t)) as u8, (60.0 * (1.0 - t)) as u8]
}

fn div_color(t: f32) -> [u8; 3] {
    // 0 で白、+1 で赤、-1 で青
    let k = t.abs().sqrt();
    let mix = |a: f32, b: f32| (a + (b - a) * k) as u8;
    if t >= 0.0 {
        [mix(245.0, 200.0), mix(245.0, 30.0), mix(245.0, 30.0)]
    } else {
        [mix(245.0, 30.0), mix(245.0, 90.0), mix(245.0, 210.0)]
    }
}

fn shade(c: [u8; 3], k: f32) -> [u8; 3] {
    c.map(|v| (v as f32 * k) as u8)
}

const LEGEND_H: usize = 26;

/// 地図 + 下端の凡例を描く RGB 画像。
struct Canvas {
    w: usize,
    h: usize,
    map_h: usize,
    px: Vec<u8>,
}

impl Canvas {
    fn base(g: &WalkGrid) -> Self {
        let (w, map_h) = (g.w.max(160), g.h);
        let h = map_h + LEGEND_H;
        let mut c = Self {
            w,
            h,
            map_h,
            px: vec![255; w * h * 3],
        };
        for y in 0..g.h {
            for x in 0..g.w {
                let i = g.idx(x, y);
                c.set(x, y, if g.walkable[i] { [200, 200, 198] } else { [30, 30, 36] });
            }
        }
        c
    }
    /// 地図座標（y は北向き）で塗る
    fn set(&mut self, x: usize, y: usize, c: [u8; 3]) {
        if x < self.w && y < self.map_h {
            let o = ((self.map_h - 1 - y) * self.w + x) * 3;
            self.px[o..o + 3].copy_from_slice(&c);
        }
    }
    /// 画像座標（y は下向き）で塗る
    fn put(&mut self, x: usize, y: usize, c: [u8; 3]) {
        if x < self.w && y < self.h {
            let o = (y * self.w + x) * 3;
            self.px[o..o + 3].copy_from_slice(&c);
        }
    }
    fn legend(&mut self, lo: f32, hi: f32, diverging: bool) {
        let top = self.map_h + 4;
        let bar_w = (self.w / 2).max(80);
        let x0 = (self.w - bar_w) / 2;
        for i in 0..bar_w {
            let t = i as f32 / (bar_w - 1) as f32;
            let c = if diverging { div_color(t * 2.0 - 1.0) } else { seq_color(t) };
            for y in top..top + 8 {
                self.put(x0 + i, y, c);
            }
        }
        let lo_s = fmt_num(lo);
        let hi_s = fmt_num(hi);
        self.text(x0.saturating_sub(text_w(&lo_s) + 4), top, &lo_s);
        self.text(x0 + bar_w + 4, top, &hi_s);
        if diverging {
            self.text(x0 + bar_w / 2 - 1, top + 10, "0");
        }
    }
    fn text(&mut self, x: usize, y: usize, s: &str) {
        let mut cx = x;
        for ch in s.chars() {
            if let Some(g) = glyph(ch) {
                for (row, bits) in g.iter().enumerate() {
                    for col in 0..3 {
                        if bits & (4 >> col) != 0 {
                            for dy in 0..2 {
                                for dx in 0..2 {
                                    self.put(cx + col * 2 + dx, y + row * 2 + dy, [20, 20, 20]);
                                }
                            }
                        }
                    }
                }
            }
            cx += 8;
        }
    }
    fn encode(&self) -> Vec<u8> {
        png::encode_rgb(self.w as u32, self.h as u32, &self.px)
    }
}

fn text_w(s: &str) -> usize {
    s.chars().count() * 8
}

fn fmt_num(v: f32) -> String {
    let a = v.abs();
    if a == 0.0 {
        "0".into()
    } else if a >= 100.0 {
        format!("{v:.0}")
    } else if a >= 1.0 {
        format!("{v:.1}")
    } else {
        format!("{v:.3}")
    }
}

/// 3x5 の数字フォント
fn glyph(c: char) -> Option<[u8; 5]> {
    Some(match c {
        '0' => [7, 5, 5, 5, 7],
        '1' => [2, 6, 2, 2, 7],
        '2' => [7, 1, 7, 4, 7],
        '3' => [7, 1, 7, 1, 7],
        '4' => [5, 5, 7, 1, 1],
        '5' => [7, 4, 7, 1, 7],
        '6' => [7, 4, 7, 5, 7],
        '7' => [7, 1, 1, 1, 1],
        '8' => [7, 5, 7, 5, 7],
        '9' => [7, 5, 7, 1, 7],
        '.' => [0, 0, 0, 0, 2],
        '-' => [0, 0, 7, 0, 0],
        '+' => [0, 2, 7, 2, 0],
        _ => return None,
    })
}

/// 区画値の取り出し（画像出力用）
pub enum ZoneMetric {
    PassesPerH,
    AvgWait,
    AvgDwell,
    PeakDensity,
}

impl ZoneStats {
    pub fn metric(&self, m: ZoneMetric) -> Vec<f32> {
        (0..self.nx * self.ny)
            .map(|z| match m {
                ZoneMetric::PassesPerH => self.passes_per_h(z),
                ZoneMetric::AvgWait => self.avg_wait(z),
                ZoneMetric::AvgDwell => self.avg_dwell(z),
                ZoneMetric::PeakDensity => self.peak_density(z),
            })
            .collect()
    }

    /// 出力ディレクトリに一式を書き出す。`before` があれば比較列と差分画像も出す。
    /// `closed` は通行止めで通れなくなったセル（差分画像の斜線用）。書いたファイル名を返す。
    pub fn export_dir(
        &self,
        g: &WalkGrid,
        dir: &std::path::Path,
        before: Option<&ZoneStats>,
        closed: &[bool],
        clock: impl Fn(f32) -> String,
    ) -> std::io::Result<Vec<String>> {
        std::fs::create_dir_all(dir)?;
        let mut files = vec![];
        let mut put = |name: &str, bytes: Vec<u8>| -> std::io::Result<()> {
            std::fs::write(dir.join(name), bytes)?;
            files.push(name.to_string());
            Ok(())
        };
        let mut csv = vec![];
        self.write_zones_csv(g, before, &mut csv)?;
        put("zones.csv", csv)?;
        let mut tl = vec![];
        self.write_timeline_csv(&clock, &mut tl)?;
        put("timeline.csv", tl)?;
        put("heatmap.png", heat_png(g, &self.heat_rate()))?;
        put("zones_passes_per_h.png", self.zone_png(g, &self.metric(ZoneMetric::PassesPerH)))?;
        put("zones_avg_wait_s.png", self.zone_png(g, &self.metric(ZoneMetric::AvgWait)))?;
        put("zones_avg_dwell_s.png", self.zone_png(g, &self.metric(ZoneMetric::AvgDwell)))?;
        if let Some(b) = before.filter(|b| b.heat.len() == self.heat.len()) {
            put("diff_heatmap.png", diff_png(g, &b.heat_rate(), &self.heat_rate(), closed))?;
            let bm = b.metric(ZoneMetric::PassesPerH);
            let am = self.metric(ZoneMetric::PassesPerH);
            if bm.len() == am.len() {
                put("diff_zones_passes_per_h.png", before_after_zone_diff(self, g, &bm, &am))?;
            }
        }
        Ok(files)
    }
}

fn before_after_zone_diff(st: &ZoneStats, g: &WalkGrid, before: &[f32], after: &[f32]) -> Vec<u8> {
    let diff: Vec<f32> = after.iter().zip(before).map(|(a, b)| a - b).collect();
    let scale = diff.iter().map(|d| d.abs()).fold(0.0, f32::max).max(1e-6);
    let mut img = Canvas::base(g);
    for y in 0..g.h {
        for x in 0..g.w {
            let i = g.idx(x, y);
            if !g.walkable[i] {
                continue;
            }
            let z = st.cell_zone[i] as usize;
            img.set(x, y, div_color(diff[z] / scale));
        }
    }
    img.legend(-scale, scale, true);
    img.encode()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zone_stats_count_passes() {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let g = WalkGrid::from_city(&m, 0.5).unwrap();
        let (mut sim, _) = crate::scenario::through_traffic(g, 100, 3).unwrap();
        let mut st = ZoneStats::new(&sim.grid, 20.0);
        for _ in 0..1200 {
            sim.step(0.05);
            st.record(&sim, std::iter::empty(), 0.05);
        }
        assert!((st.elapsed - 60.0).abs() < 0.01);
        assert_eq!(st.timeline.len(), 1);
        let passes: u32 = st.ped_passes.iter().sum();
        assert!(passes >= 100, "初期配置の区画も 1 回と数える: {passes}");
        let mut csv = vec![];
        st.write_zones_csv(&sim.grid, Some(&st.clone()), &mut csv).unwrap();
        let text = String::from_utf8(csv).unwrap();
        assert!(text.lines().count() > 10);
        assert!(text.lines().nth(1).unwrap().split(',').count() == 24);
        let dir = std::env::temp_dir().join(format!("citysim-report-{}", std::process::id()));
        let files = st
            .export_dir(&sim.grid, &dir, Some(&st.clone()), &[], |t| format!("{t}"))
            .unwrap();
        assert!(files.contains(&"diff_heatmap.png".to_string()));
        let _ = std::fs::remove_dir_all(dir);
    }
}
