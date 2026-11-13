#!/usr/bin/env bash
# ビューアの Web 版（wasm32 + WebGL2）を dist/ に作る。
#   必要なもの: rustup target add wasm32-unknown-unknown
#               cargo install wasm-bindgen-cli --version <Cargo.lock の wasm-bindgen と同じ>
#   任意:       wasm-opt（binaryen）があればサイズを縮める
# 確認: python3 -m http.server -d dist 8080 → http://localhost:8080/
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${1:-dist}

cargo build --profile web -p plateau-crowd-viewer --target wasm32-unknown-unknown
rm -rf "$OUT"
mkdir -p "$OUT"
wasm-bindgen --target web --no-typescript --out-dir "$OUT" \
    target/wasm32-unknown-unknown/web/plateau-crowd-viewer.wasm
if command -v wasm-opt >/dev/null; then
    wasm-opt -Os --enable-bulk-memory --enable-nontrapping-float-to-int --enable-sign-ext \
        --enable-mutable-globals --enable-reference-types \
        "$OUT/plateau-crowd-viewer_bg.wasm" -o "$OUT/plateau-crowd-viewer_bg.wasm"
fi
cp crates/viewer/web/index.html "$OUT/"
cp crates/viewer/web/fonts/OFL.txt "$OUT/font-license.txt"
ls -la "$OUT"
