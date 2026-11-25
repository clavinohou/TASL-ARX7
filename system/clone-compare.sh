#!/bin/bash
# Compare file counts per folder between the running system and the Fanxiang copy,
# then wait for ~arx/clone/poweroff-ok (created after review) and power off.
# Run as root on the robot.
set -uo pipefail
DST=$(readlink -f /dev/disk/by-id/nvme-Fanxiang_S660_500GB_26030313611001145)
NEW=/mnt/newroot
FLAG=/home/arx/clone/poweroff-ok
rm -f "$FLAG"

mount -o ro "${DST}p2" "$NEW" || { echo "!!! mount failed"; exit 1; }
echo "inodes used: source $(df --output=iused / | tail -1)  copy $(df --output=iused "$NEW" | tail -1)"

count() {  # $1 = root; prints "<count> <dir>" for each depth-1 and depth-2 dir
  (cd "$1" && find . -xdev -mindepth 1 2>/dev/null | cut -d/ -f2-3 | sort | uniq -c)
}
count / > /tmp/src.cnt
count "$NEW" > /tmp/dst.cnt
echo "-- folders where the source has 50+ more entries than the copy:"
join -1 2 -2 2 -a 1 -e 0 -o 0,1.1,2.1 <(awk '{print $1, $2}' /tmp/src.cnt | sort -k2) \
                                     <(awk '{print $1, $2}' /tmp/dst.cnt | sort -k2) |
  awk '$2 - $3 >= 50 {printf "  %-45s source %7d  copy %7d  missing %7d\n", $1, $2, $3, $2 - $3}'
echo "-- totals: source $(find / -xdev 2>/dev/null | wc -l)  copy $(find "$NEW" -xdev 2>/dev/null | wc -l)"
umount "$NEW"
echo "=== REPORT DONE — waiting up to 20 min for $FLAG"

for i in $(seq 1 240); do
  if [ -f "$FLAG" ]; then echo "=== powering off"; sync; sleep 3; systemctl poweroff; exit 0; fi
  sleep 5
done
echo "=== no go-ahead in 20 min — staying on"
