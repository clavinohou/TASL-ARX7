#!/bin/bash
# Reinstall packages whose files an earlier fsck on the Kingston moved into /lost+found
# (found by dpkg --verify after the clone). Run as root.
set -uo pipefail
say() { printf '\n=== %s  %s\n' "$(date +%T)" "$*"; }
[ "$(id -u)" -eq 0 ] || { echo "run as root"; exit 1; }
export DEBIAN_FRONTEND=noninteractive

say "reinstalling packages with missing files"
for p in libboost1.74-dev libaprutil1-dev libb64-dev libb64-0d libconsole-bridge-dev \
         libpocoutil80 libpoconet80 libpocomongodb80 libpocodatasqlite80 \
         libpocodatapostgresql80 libpocodataodbc80 libpocodatamysql80 libodbc2; do
  if apt-get install --reinstall -y -q "$p" >/tmp/reinstall-$p.log 2>&1; then
    echo "  ok   $p"
  else
    echo "  FAIL $p: $(tail -2 /tmp/reinstall-$p.log | tr '\n' ' ')"
  fi
done

say "checks (as root, so nothing is hidden by permissions)"
dpkg --verify 2>/dev/null | awk '$1 == "missing" && $2 != "c" {print $NF}' > /tmp/missing-after.txt
echo "  files still missing from installed packages: $(wc -l < /tmp/missing-after.txt)"
head -20 /tmp/missing-after.txt
dpkg --verify 2>/dev/null | awk '$1 ~ /5/ && $2 != "c" {print $NF}' > /tmp/changed-after.txt
echo "  non-config files whose contents differ from the package: $(wc -l < /tmp/changed-after.txt)"
head -20 /tmp/changed-after.txt
[ -f /usr/include/boost/version.hpp ] && echo "  ok   boost headers" || echo "  FAIL boost headers"
say "DONE"
