#!/bin/bash
archs=(x86_64-unknown-linux-gnu aarch64-unknown-linux-gnu)
for i in "${archs[@]}"
do
	cargo deb --target=$i -o debs
done
