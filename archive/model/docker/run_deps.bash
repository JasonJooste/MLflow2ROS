#!/usr/bin/env bash
set -e

# Collect dependencies of the project
# Take rosdep output, remove comments and batch apt commands for faster installation
SCRIPTPATH="$(dirname "$0")"
"$SCRIPTPATH/run_dev.bash" bash -c "set -o pipefail \
  && rosdep update \
  && rosdep install --default-yes --from-paths /workspace --ignore-src --reinstall --simulate -r | sort > rosdep.out \
  && printf \"#!/usr/bin/env bash\n\" > install_deps.bash \
  && cat rosdep.out | sed \"/apt-get install -y/d\" | sed \"/^#/d\" >> install_deps.bash \
  && printf \"apt-get install -y \" >> install_deps.bash \
  && cat rosdep.out | sed -n \"/apt-get install -y/p\" | awk '{print \$4}' ORS=' ' >> install_deps.bash \
  && chmod +x install_deps.bash \
  && cat install_deps.bash"
