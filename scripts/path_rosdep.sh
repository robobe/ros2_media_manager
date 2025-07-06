cat <<'EOF' > /etc/ros/rosdep/sources.list.d/30-custom.list 
yaml file:///workspace/rosdep.yaml
EOF

# rosdep update --rosdistro humble