#!/usr/bin/env bash
# Fetch vendored third-party dependencies into vendor/.
# Idempotent: skips repos that are already present and at the pinned SHA.
# See DEPENDENCIES.md for the canonical list, upstream URLs, and pinned SHAs.

set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
vendor_dir="${repo_root}/vendor"
mkdir -p "${vendor_dir}"

# name|url|sha
deps=(
    "crazyflie-link-cpp|https://github.com/bitcraze/crazyflie-link-cpp.git|598dea123326652e8e82ea642abaff3496e922fe"
    "mcap|https://github.com/foxglove/mcap.git|c3cab6bd3ce79199e362766daec3a4689f3a0335"
    "doctest|https://github.com/doctest/doctest.git|d44d4f6e66232d716af82f00a063759e9d0e50d6"
)

clone_at_sha() {
    local name="$1" url="$2" sha="$3"
    local dest="${vendor_dir}/${name}"

    if [[ -d "${dest}/.git" ]]; then
        local current
        current="$(git -C "${dest}" rev-parse HEAD)"
        if [[ "${current}" == "${sha}" ]]; then
            echo "[ok] ${name} already at ${sha:0:12}"
            return 0
        fi
        echo "[update] ${name}: ${current:0:12} -> ${sha:0:12}"
        git -C "${dest}" fetch --quiet origin "${sha}"
        git -C "${dest}" -c advice.detachedHead=false checkout --quiet "${sha}"
        return 0
    fi

    echo "[clone] ${name} @ ${sha:0:12}"
    git clone --quiet "${url}" "${dest}"
    git -C "${dest}" -c advice.detachedHead=false checkout --quiet "${sha}"
}

for entry in "${deps[@]}"; do
    IFS='|' read -r name url sha <<< "${entry}"
    clone_at_sha "${name}" "${url}" "${sha}"
done

echo "vendor bootstrap complete."
