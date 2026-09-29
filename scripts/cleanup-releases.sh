# shellcheck shell=bash
# Delete outdated GitHub releases and prereleases.
#   - keeps every major.minor line, but only N newest patches inside it
#   - deletes prereleases that already have a stable counterpart
#   - keeps the N newest remaining prereleases (globally, or per base version)
#
# This file is NOT executable on purpose. Run it explicitly:
#   bash scripts/cleanup-releases.sh [options]
#
# Requires: gh (authenticated), jq.  Dry-run is the default.
set -euo pipefail

KEEP_PATCHES=3
KEEP_PRERELEASES=5
PRE_SCOPE=global        # global | base
DELETE_TAGS=false       # git tags are kept unless --delete-tags is given
LIST_LIMIT=1000
DRY_RUN=true
ASSUME_YES=false
SHOW_ALL=true           # false => only rows that would be deleted
PROTECT_TAG=""
REPO="${GH_REPO:-}"

# environment overrides (useful in CI)
[ -n "${CLEANUP_DRY_RUN:-}" ] && { [ "$CLEANUP_DRY_RUN" = "false" ] && DRY_RUN=false || DRY_RUN=true; }

usage() {
  cat <<'EOF'
Usage: bash scripts/cleanup-releases.sh [options]

  -R, --repo OWNER/REPO      target repository (default: current git remote)
      --keep-patches N       newest patch releases per major.minor line (default: 3)
      --keep-prereleases N   prereleases to keep                        (default: 5)
      --prerelease-scope S   global = N newest prereleases overall (default)
                             base   = N newest of the highest base version only
                                      (use only with suffixed tags like 1.2.0-rc.1)
      --protect TAG          never delete this tag
      --limit N              max releases to fetch (default: 1000)
      --delete-tags          also delete the git tag of each removed release
      --no-delete-tags       keep the git tags (default)
      --only-deletions       hide the "keep" rows from the report
      --execute              actually delete (default is dry-run)
  -y, --yes                  skip the confirmation prompt
  -h, --help                 show this help
EOF
}

die() { echo "Error: $*" >&2; exit 1; }
is_uint() { case "${1:-}" in ''|*[!0-9]*) return 1;; *) return 0;; esac; }

while [ $# -gt 0 ]; do
  case "$1" in
    -R|--repo)           REPO="${2:-}"; shift 2 ;;
    --keep-patches)      KEEP_PATCHES="${2:-}"; shift 2 ;;
    --keep-prereleases)  KEEP_PRERELEASES="${2:-}"; shift 2 ;;
    --prerelease-scope)  PRE_SCOPE="${2:-}"; shift 2 ;;
    --protect)           PROTECT_TAG="${2:-}"; shift 2 ;;
    --limit)             LIST_LIMIT="${2:-}"; shift 2 ;;
    --delete-tags)       DELETE_TAGS=true; shift ;;
    --no-delete-tags)    DELETE_TAGS=false; shift ;;
    --only-deletions)    SHOW_ALL=false; shift ;;
    --execute)           DRY_RUN=false; shift ;;
    --dry-run)           DRY_RUN=true; shift ;;
    -y|--yes)            ASSUME_YES=true; shift ;;
    -h|--help)           usage; exit 0 ;;
    *) usage >&2; die "unknown option: $1" ;;
  esac
done

# --- preconditions ---
command -v gh >/dev/null || die "gh (GitHub CLI) not found - https://cli.github.com"
command -v jq >/dev/null || die "jq not found"
gh auth status >/dev/null 2>&1 || die "gh is not authenticated - run: gh auth login"
is_uint "$KEEP_PATCHES" && [ "$KEEP_PATCHES" -ge 1 ] || die "--keep-patches must be an integer >= 1"
is_uint "$KEEP_PRERELEASES" || die "--keep-prereleases must be an integer >= 0"
is_uint "$LIST_LIMIT" && [ "$LIST_LIMIT" -ge 1 ] || die "--limit must be an integer >= 1"
case "$PRE_SCOPE" in global|base) ;; *) die "--prerelease-scope must be 'global' or 'base'";; esac

if [ -z "$REPO" ]; then
  REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner 2>/dev/null) \
    || die "no repository detected - use --repo OWNER/REPO"
fi
case "$REPO" in */*) ;; *) die "--repo must be OWNER/REPO, got '$REPO'";; esac

TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
RELEASES="$TMP/releases.json"
REPORT="$TMP/report.tsv"

echo "Repository : $REPO"
echo "Mode       : $([ "$DRY_RUN" = true ] && echo 'dry-run' || echo 'LIVE - deleting')"
echo "Retention  : $KEEP_PATCHES patches per major.minor line, $KEEP_PRERELEASES prereleases ($PRE_SCOPE scope)"
echo "Git tags   : $([ "$DELETE_TAGS" = true ] && echo 'deleted with the release' || echo 'kept')"
[ -n "$PROTECT_TAG" ] && echo "Protected  : $PROTECT_TAG"
echo

gh release list --repo "$REPO" --limit "$LIST_LIMIT" \
  --json tagName,isDraft,isPrerelease > "$RELEASES"

# --- classify every release: section, group, tag, kind, action, reason ---
jq -r \
  --argjson keep_patch "$KEEP_PATCHES" \
  --argjson keep_pre "$KEEP_PRERELEASES" \
  --arg scope "$PRE_SCOPE" \
  --arg protect "$PROTECT_TAG" '
  def parse:
    . as $r
    | ( $r.tagName
        | capture("^[vV]?(?<ma>[0-9]+)\\.(?<mi>[0-9]+)\\.(?<pa>[0-9]+)(?:-(?<pre>[0-9A-Za-z.-]+))?(?:\\+[0-9A-Za-z.-]+)?$")
        // null ) as $m
    | if $m == null then empty
      else
        { tag: $r.tagName,
          major: ($m.ma|tonumber), minor: ($m.mi|tonumber), patch: ($m.pa|tonumber),
          pre: $m.pre,
          isPre: ($r.isPrerelease or ($m.pre != null)) }
        | . + { base: "\(.major).\(.minor).\(.patch)", line: "\(.major).\(.minor)" }
      end;

  # numeric identifiers rank below alphanumeric ones; stable > prerelease
  def prekey:
    if .pre == null then [1]
    else [0] + (.pre | split(".")
                | map(if test("^[0-9]+$") then [0, tonumber] else [1, .] end))
    end;
  def vkey: [.major, .minor, .patch] + prekey;

  [ .[] | select(.isDraft | not) ]                    as $all
  | [ $all[] | parse ]                                as $items
  | [ $items[] | .tag ]                               as $known
  | [ $all[] | .tagName | select(($known | index(.)) == null) ] as $skipped
  | [ $items[] | select(.isPre | not) ]               as $stable
  | [ $items[] | select(.isPre) ]                     as $pre
  | [ $stable[] | .base ]                             as $sbases

  # (1) prereleases whose version already exists as a stable release
  | [ $pre[] | select(.base | IN($sbases[])) ]        as $superseded
  | ( [ $pre[] | select(.base | IN($sbases[]) | not) ]
      | sort_by(vkey) | reverse )                     as $ranked

  # (2) retention scope: all remaining prereleases, or only the highest base
  | ( if $scope == "base" and ($ranked|length) > 0 then $ranked[0].base else null end ) as $topbase
  | ( if $scope == "base" then [ $ranked[] | select(.base == $topbase) ] else $ranked end ) as $candidates
  | ( if $scope == "base" then [ $ranked[] | select(.base != $topbase) ] else []     end ) as $oldbase
  | ( $candidates | .[$keep_pre:] )                   as $extrapre

  # (3) stable: keep every major.minor line, drop old patches inside it
  | ( [ $stable | group_by(.line)[] | sort_by(.patch) | reverse | .[$keep_patch:] ]
      | flatten )                                     as $oldpatch

  # deletion map: tag -> reason
  | ( [ ($superseded[] | {tag, reason:"superseded by stable \(.base)"}),
        ($oldbase[]    | {tag, reason:"older base version (newest upcoming: \($topbase))"}),
        ($extrapre[]   | {tag, reason:(if $scope == "base"
                                       then "only \($keep_pre) newest prereleases of \(.base) kept"
                                       else "only \($keep_pre) newest prereleases kept" end)}),
        ($oldpatch[]   | {tag, reason:"only \($keep_patch) newest patches of \(.line) kept"}) ]
      | unique_by(.tag)
      | map(select(.tag != $protect))
      | map({key: .tag, value: .reason}) | from_entries )  as $del

  # keep reasons incl. rank
  | ( [ $candidates[:$keep_pre] | to_entries[]
        | {key: .value.tag, value: "prerelease #\(.key + 1) of \($keep_pre) kept"} ]
      | from_entries )                                as $prekeep
  | ( [ $stable | group_by(.line)[]
        | (sort_by(.patch) | reverse | .[:$keep_patch] | to_entries[]
           | {key: .value.tag,
              value: "patch #\(.key + 1) of \($keep_patch) in line \(.value.line)"}) ]
      | from_entries )                                as $stkeep

  | [ ( $stable | sort_by(vkey) | reverse | .[]
        | { sect:"1", group:.line, tag:.tag, kind:"release",
            action: (if .tag == $protect then "KEEP"
                     elif $del[.tag] then "DELETE" else "KEEP" end),
            reason: (if .tag == $protect then "protected (triggering release)"
                     elif $del[.tag] then $del[.tag]
                     else ($stkeep[.tag] // "kept") end) } ),
      ( $pre | sort_by(vkey) | reverse | .[]
        | { sect:"2", group:.base, tag:.tag, kind:"prerelease",
            action: (if .tag == $protect then "KEEP"
                     elif $del[.tag] then "DELETE" else "KEEP" end),
            reason: (if .tag == $protect then "protected (triggering release)"
                     elif $del[.tag] then $del[.tag]
                     else ($prekeep[.tag] // "kept") end) } ),
      ( $skipped[]
        | { sect:"3", group:"-", tag:., kind:"unknown", action:"SKIP",
            reason:"tag is not semver - never touched" } ) ]
  | .[] | [.sect, .group, .tag, .kind, .action, .reason] | @tsv
' "$RELEASES" > "$REPORT"

# --- render report ---
render() {  # $1=section  $2=title  $3=first column header
  local rows
  rows=$(awk -F'\t' -v s="$1" '$1==s' "$REPORT" || true)
  if [ "$SHOW_ALL" != true ] && [ "$1" != "3" ]; then
    rows=$(printf '%s\n' "$rows" | awk -F'\t' '$5=="DELETE"' || true)
  fi
  [ -n "$rows" ] || return 0
  echo "### $2"; echo
  if [ "$1" = "3" ]; then
    echo "| Tag | Reason |"; echo "| --- | --- |"
    printf '%s\n' "$rows" | while IFS=$'\t' read -r _ _ tag _ _ reason; do
      printf '| `%s` | %s |\n' "$tag" "$reason"
    done
  else
    printf '| %s | Tag | Type | Action | Reason |\n' "$3"
    echo "| --- | --- | --- | --- | --- |"
    printf '%s\n' "$rows" | while IFS=$'\t' read -r _ group tag kind action reason; do
      case "$action" in
        DELETE) label='**DELETE**' ;;
        SKIP)   label='skip' ;;
        *)      label='keep' ;;
      esac
      printf '| %s | `%s` | %s | %s | %s |\n' "$group" "$tag" "$kind" "$label" "$reason"
    done
  fi
  echo
}

render 1 "Stable releases" "Line"
render 2 "Prereleases"     "Base"
render 3 "Ignored tags"    "-"

kept=$(awk -F'\t'    '$5=="KEEP"'   "$REPORT" | wc -l | tr -d ' ')
todel=$(awk -F'\t'   '$5=="DELETE"' "$REPORT" | wc -l | tr -d ' ')
skipped=$(awk -F'\t' '$5=="SKIP"'   "$REPORT" | wc -l | tr -d ' ')
echo "Summary: $kept kept · $todel to delete · $skipped ignored"

# write the report to the GitHub Actions job summary when available
if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    echo "## Release cleanup ($REPO)"
    echo
    echo "Mode: \`$([ "$DRY_RUN" = true ] && echo dry-run || echo execute)\` · \
Retention: $KEEP_PATCHES patches / $KEEP_PRERELEASES prereleases ($PRE_SCOPE)"
    echo
    SHOW_ALL="$SHOW_ALL" render 1 "Stable releases" "Line"
    render 2 "Prereleases" "Base"
    render 3 "Ignored tags" "-"
    echo "Summary: $kept kept · $todel to delete · $skipped ignored"
  } >> "$GITHUB_STEP_SUMMARY"
fi

[ "$todel" -gt 0 ] || { echo "Nothing to delete."; exit 0; }

if [ "$DRY_RUN" = true ]; then
  echo "Dry-run: nothing was deleted. Re-run with --execute to apply."
  exit 0
fi

if [ "$ASSUME_YES" != true ] && [ -t 0 ]; then
  printf 'Delete %s release(s) from %s? [y/N] ' "$todel" "$REPO"
  read -r answer
  case "$answer" in [yY]|[yY][eE][sS]) ;; *) echo "Aborted."; exit 1 ;; esac
fi

awk -F'\t' '$5=="DELETE" { print $3 "\t" $4 }' "$REPORT" > "$TMP/del.tsv"
deleted=0; failed=0
while IFS=$'\t' read -r tag kind; do
  [ -n "${tag:-}" ] || continue
  args=(--repo "$REPO" --yes)
  if [ "$DELETE_TAGS" = true ]; then args+=(--cleanup-tag); fi
  if out=$(gh release delete "$tag" "${args[@]}" 2>&1); then
    deleted=$((deleted + 1)); echo "deleted: $tag ($kind)"
  else
    failed=$((failed + 1));  echo "FAILED : $tag - $out" >&2
  fi
done < "$TMP/del.tsv"

echo
echo "Result: $deleted deleted · $failed failed"
[ "$failed" -eq 0 ] || exit 1
