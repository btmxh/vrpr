#!/bin/sh

parallel --bar '
    cargo run --release "{}" > "{}.result.jsonc"
' ::: ~/Downloads/Instance/WithTimeWindows/$@*.json

