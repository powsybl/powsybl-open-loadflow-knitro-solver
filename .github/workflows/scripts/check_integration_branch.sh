#!/bin/bash

repo=$1
core_version=$2

# Add "-SNAPSHOT" to powsybl-core version if not already there
core_snapshot_version=$(echo "$core_version" | grep -q SNAPSHOT && echo "$core_version" || echo "$core_version-SNAPSHOT")

# Check if an integration branch exists
INTEGRATION_BRANCH=$(git ls-remote --heads "$repo" | grep -E "refs/heads/integration/powsyblcore-$core_snapshot_version" | sed 's/.*refs\/heads\///')
if [ -n "$INTEGRATION_BRANCH" ]; then
    echo "Integration branch found: $INTEGRATION_BRANCH"
    echo "INTEGRATION_BRANCH=$INTEGRATION_BRANCH" >> "$GITHUB_ENV"
else
    echo "No integration branch found, using main branch"
    echo "INTEGRATION_BRANCH=main" >> "$GITHUB_ENV"
fi
