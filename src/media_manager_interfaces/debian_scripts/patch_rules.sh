#!/bin/sh

cat <<'EOF' >> debian/rules

override_dh_builddeb:
	dh_builddeb --destdir=/workspace/debs
EOF