#!/usr/bin/env sh
# Fails (exit 1) if anything that would be published looks like personal data:
# IPv4 / MAC addresses, e-mails (except noreply identities), API tokens / private keys,
# or host-specific absolute paths. Scans tracked + untracked files (respecting .gitignore,
# so target/ and dist/ are skipped) and all commit author/committer/message metadata.
set -eu
cd "$(dirname "$0")/.."

# This script contains the patterns themselves. CLAUDE.md / TASK.md are operator instructions
# for coding agents, not part of the app; scrub or delete them before publishing.
skip='^(tools/privacy_check\.sh|CLAUDE\.md|TASK\.md)$'

ip='\b([0-9]{1,3}\.){3}[0-9]{1,3}\b'
mac='\b([0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}\b'
email='[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}'
token='(ghp_|gho_|ghs_|github_pat_|sk-|xox[abpr]-|AKIA|AIza)[A-Za-z0-9_-]{10,}|-----BEGIN [A-Z ]*PRIVATE KEY-----'
paths='/opt/data|/volume[0-9]+|/home/[A-Za-z0-9_]+|/Users/[A-Za-z0-9_]+|[A-Za-z]:\\\\Users'
pattern="$ip|$mac|$email|$token|$paths"
allow='@users\.noreply\.github\.com|^[^ ]*noreply@anthropic\.com$|:noreply@anthropic\.com$'

status=0

files=$(git ls-files -co --exclude-standard | grep -Ev "$skip" || true)
hits=$(printf '%s\n' "$files" | tr '\n' '\0' | xargs -0 grep -noIE "$pattern" -- 2>/dev/null | grep -Ev "$allow" || true)
if [ -n "$hits" ]; then
  echo "privacy_check: suspicious content in files:"
  echo "$hits"
  status=1
fi

meta=$(git log --all --format='%an <%ae>%n%cn <%ce>%n%B' 2>/dev/null | grep -oE "$pattern" | grep -Ev "$allow" || true)
if [ -n "$meta" ]; then
  echo "privacy_check: suspicious content in commit metadata:"
  echo "$meta"
  status=1
fi

[ "$status" -eq 0 ] && echo "privacy_check: clean ($(printf '%s\n' "$files" | wc -l | tr -d ' ') files, $(git rev-list --all --count) commits)"
exit "$status"
