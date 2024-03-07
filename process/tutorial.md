# Versioning

Gazebo version numbers are composed of three numbers, such as 1.9.2, that represent MAJOR.MINOR.PATCH

As of Gazebo 2.0.0, each number is incremented according to:

 1. MAJOR is increased when breaking ABI/API changes are introduced.
  2. MINOR is increased when ABI/API compatible new features are introduced.
   3. PATCH is increased when ABI/API compatible bug fixes are introduced.

   This means all 2.x.x versions will be ABI/API compatible.

# Gazebo Branches and Pull-Requests

   The [Gazebo repository](https://github.com/osrf/gazebo) is located on [GitHub](http://github.com). 

## Branches

   1. ''master'': Contains all code that breaks ABI/API compatibility with the current released version

   2. ''gazeboX'': Contains code for Gazebo version X, where X is the MAJOR version. Only bugs fixes and features that do not break ABI/API may be submitted to this branch.

   3. All other branches are unstable development branches.

##Rules

   1. All pull requests must contain an  ABI/API compatibility report.

   2. All pull requests to 'gazeboX' must be associated with an issue.

   3. 2 or more approvals are required for a pull request to be merged.

   4. All pull requests must pass the [code checker](/tutorials?tut=contrib_code&cat=development#CodeCheck), contain no build warnings, and [pass all the tests](/tutorials?tut=contrib_code&cat=development#WriteTests).
     
##Distinction between bugs and features

   As mentioned above, new features cause a bump to the minor version number, and only bug fixes can be included in patch releases. Since there is not always consensus on the distinction between features and bug fixes, we use the following guidelines to decide:

   1. Style / whitespace fixes are not bugs unless they fix the code_check.sh results.

   2. Adding a new test is a feature, not a bug fix.

# OSRF Internal Development Process

   The steps listed below detail how a Gazebo release is developed.
   This process is enforced by the core Gazebo team, which currently resides at [OSRF](http://osrfoundation.org).

## Step 1: New features and/or bug fixes

   A set of features and functionality are decided upon. These features are chosen based on public demand, and any contractual obligations. Feature freeze, code freeze, and release dates are also scheduled at this time along with distribution of work to key people.

## Step 2: Implementation and Review

   This is the fun part. The features decided upon in Step 1 are implemented, documentation written, and tests created. All code must pass our tests, static code checking, and be submitted as a pull-request through GitHub.

## Step 3: Feature Freeze

   A branch is created, typically named with the upcoming major.minor number. This branch will eventually become the final Gazebo release. No new features are allowed in this branch from this point on. Only bug fixes will be accepted.

## Step 4: Code Freeze

   At this point almost all of the major bugs should be ironed out of the release. Other issues should be tracked on the GitHub issue tracker, and documented. Only very critical changes are allowed in the release branch at this time.

## Step 5: Release

   A Debian package and source tarball is generated from the release branch. Documentation on the Gazebo website is updated.

