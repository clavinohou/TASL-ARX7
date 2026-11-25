#!/bin/bash
# Reinstall system and Python packages whose files were lost to bad sectors on the
# old Kingston drive during the clone to the Fanxiang (2026-11-24). Run as root.
set -uo pipefail
say() { printf '\n=== %s  %s\n' "$(date +%T)" "$*"; }
[ "$(id -u)" -eq 0 ] || { echo "run as root"; exit 1; }
export DEBIAN_FRONTEND=noninteractive

say "apt update"
apt-get update -q || echo "!!! apt update failed"

say "reinstalling apt packages"
for p in libutempter0 pulseaudio libpoco-dev libicu-dev mysql-common \
         linux-headers-6.8.0-40-generic linux-hwe-6.8-headers-6.8.0-40; do
  if apt-get install --reinstall -y -q "$p" >/tmp/reinstall-$p.log 2>&1; then
    echo "  ok   $p"
  else
    echo "  FAIL $p: $(tail -2 /tmp/reinstall-$p.log | tr '\n' ' ')"
  fi
done

say "reinstalling pip packages in /usr/local (same versions)"
python3 -m pip install -q --force-reinstall --no-deps \
  scipy==1.15.3 matplotlib==3.10.5 kiwisolver==1.4.8 && echo "  ok   pip" || echo "  FAIL pip"

say "removing old firefox revision 7720 (its .snap file was lost)"
snap remove firefox --revision=7720 && echo "  ok   snap" || echo "  FAIL snap"
systemctl reset-failed

say "checks"
echo "-- files still missing from installed packages (dpkg --verify):"
dpkg --verify 2>/dev/null | awk '$1 == "missing"' | grep -v ' c /' | head -40 || true
echo "-- (end of list)"
python3 -c "import scipy.signal, scipy.linalg, scipy.special, scipy.stats, scipy.interpolate, matplotlib.pyplot, kiwisolver; print('  ok   python imports')" 2>&1 | tail -1
tmux -V >/dev/null 2>&1 && echo "  ok   tmux runs" || echo "  FAIL tmux"
update-alternatives --query my.cnf >/dev/null 2>&1 && echo "  ok   my.cnf alternative" || echo "  FAIL my.cnf alternative"
echo "-- failed units:"; systemctl --failed --no-legend || true
say "DONE"
