#!/bin/bash
# Lets phones open http://maavnetwork.local and https://maavnetwork.local with no port, by
# redirecting ports 80 and 443 to the hub's 8080 and 8443. The hub itself keeps running as
# a normal user, and agents keep using 8080.
#
#   sudo webapp/tools/serve_on_80_and_443.sh            # add the redirects (safe to repeat)
#   sudo webapp/tools/serve_on_80_and_443.sh --persist  # and keep them across reboots
#   sudo webapp/tools/serve_on_80_and_443.sh --remove   # take them away again
#
# Run it on the machine the hub runs on: Drone 1 in the field, or the SITL VM.
set -euo pipefail

if [ "$(id -u)" != 0 ]; then
    echo "Run this with sudo." >&2
    exit 1
fi

action=add
case "${1:-}" in
    --remove) action=remove ;;
    --persist) action=persist ;;
    '') ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
esac

rule() {
    # rule OP CHAIN [MATCH...]: one redirect in the nat table. -C checks for it, -A adds it,
    # -D deletes it. -p tcp has to come before --dport.
    local op=$1 chain=$2
    shift 2
    iptables -t nat "$op" "$chain" -p tcp "$@" -j REDIRECT --to-ports "$to"
}

for pair in 80:8080 443:8443; do
    from=${pair%%:*}
    to=${pair##*:}
    # PREROUTING is for phones and other machines; OUTPUT is for a browser on this machine itself.
    for match in "PREROUTING --dport $from" "OUTPUT -o lo --dport $from"; do
        # shellcheck disable=SC2086
        set -- $match
        chain=$1
        shift
        if [ "$action" = remove ]; then
            while rule -C "$chain" "$@" 2>/dev/null; do rule -D "$chain" "$@"; done
        elif ! rule -C "$chain" "$@" 2>/dev/null; then
            rule -A "$chain" "$@"
        fi
    done
done

if [ "$action" = remove ]; then
    echo "Removed the redirects from 80 and 443."
    exit 0
fi
echo "Ports 80 and 443 now reach the hub's 8080 and 8443."

if [ "$action" = persist ]; then
    if command -v netfilter-persistent >/dev/null; then
        netfilter-persistent save
        echo "Saved: the redirects come back after a reboot."
    else
        echo "To keep them across reboots: sudo apt install iptables-persistent, then run this again with --persist." >&2
        exit 1
    fi
else
    echo "They last until the next reboot. Run again with --persist to keep them."
fi
