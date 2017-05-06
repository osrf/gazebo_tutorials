# Code review

You've come a long way. On previous tutorials you learned how to make
changes to Gazebo's codebase locally in your computer.

On this last tutorial of the series, you will learn how to integrate
your code into the official Gazebo repository!

## Choosing an issue

Before starting to write code, it's a good idea to find out if anyone is
already working on that, whether anyone has tried it before but failed,
or simply if it makes sense.

On Bitbucket, there is a list of "issues", which may be bugs or feature
requests for each repository. Here are the links to the issue lists
for each Gazebo-related repository:

* [All Gazebo issues](https://bitbucket.org/osrf/gazebo/issues)
* [All SDFormat issues](https://bitbucket.org/osrf/sdformat/issues)
* [All Ignition Math issues](https://bitbucket.org/ignitionrobotics/ign-math/issues)
* [All Ignition Transport issues](https://bitbucket.org/ignitionrobotics/ign-transport/issues)
* [All Ignition Messages issues](https://bitbucket.org/ignitionrobotics/ign-msgs/issues)

### Issues for new developers

Among these issues, the Gazebo core team marks some of them with the keyword
`IssueForNewDevelopers`. These are issues which are considered good first
issues for first time contributors like you ;) Take a look at the list and see
if you find anything that's interesting to you.

* [Gazebo issues for new developers](https://bitbucket.org/osrf/gazebo/issues?q=issuefornewdevelopers)
* [SDFormat issues for new developers](https://bitbucket.org/osrf/sdformat/issues?q=issuefornewdevelopers)
* [Ignition Math issues for new developers](https://bitbucket.org/ignitionrobotics/ign-math/issues?q=issuefornewdevelopers)
* [Ignition Transport issues for new developers](https://bitbucket.org/ignitionrobotics/ign-transport/issues?q=issuefornewdevelopers)
* [Ignition Messages issues for new developers](https://bitbucket.org/ignitionrobotics/ign-msgs/issues?q=issuefornewdevelopers)

Feel free to leave a comment on one of those issues saying that you're
considering tackling it and asking for additional information as needed.
Some issues might seem too large, and it's ok to only partially solve
an issue if that solution makes sense by itself.

### Other issues

If you have something in mind to work on, it's a very good idea to search
for a few related keywords on the issue tracker first. Some possible
results of your search are:

1. The issue has already been fixed for the upcoming release

    That's good to know, right? Now you can look for another issue to tackle!

1. There is someone actively working on it already

    Feel free to comment on the issue offering your help with anything they
    might see fit.

1. Someone has worked on it in the past but has left it in an incomplete state

    Feel free to post a comment asking for clarification on what's the current
    status of their work, whether they're planning to keep working on it, why
    they stopped... Ask anything which might help you pick up from where they
    left.

1. There are no issues related to that

    In this case, it's recommended you create a new issue yourself, explaining
    why you think that's an issue and how you plan on fixing it. Someone from
    the community or from the Gazebo core team might comment on it shortly,
    with tips on how to proceed, or maybe pointing you to other related issues
    you might have missed.

## Fixing the issue

So now you've gone through the issue trackers above and chose an issue to
work on. Before you start to code, it's a good idea to create a new
branch for this code to live.

1. Let's go back to our workspace:

        cd ~/code/gazebo

1. We had previously made some changes to files, right? You can check with the
`hg diff` command, which shows the "difference" in your workspace:

        hg diff

1. If you've been following these tutorials, all you'll have is:

        diff -r 7d7c37d66d00 gazebo/gui/TimeWidget.cc
        --- a/gazebo/gui/TimeWidget.cc  Wed May 03 14:44:20 2017 +0000
        +++ b/gazebo/gui/TimeWidget.cc  Fri May 05 17:27:20 2017 -0700
        @@ -134,7 +134,7 @@
               QSizePolicy::Minimum));
           frameLayout->addWidget(playToolbar);

        -  this->dataPtr->realTimeFactorLabel = new QLabel(tr("Real Time Factor:"));
        +  this->dataPtr->realTimeFactorLabel = new QLabel(tr("RTF:"));
           frameLayout->addWidget(this->dataPtr->realTimeFactorLabel);
           frameLayout->addWidget(this->dataPtr->percentRealTimeEdit);

    Lines preceeded by `-` have been removed, and those preceeded by `+` have
    been added.

1. We don't want that anymore, so let's clear our workspace with the following
command:

        hg up -C

    Beware that all your changes are lost when you do this!

1. Now if you check the diff again, there will be nothing!

1. Now we're ready to create our branch. First let's take a look at which
branch we're currently at with the `hg branch` command:

        hg branch

1. If you've been following the tutorials, you should now be at `default`.

1. To create a new branch we pass the branch name to the `hg branch` command.
This creates a new branch as a copy of the branch we're currently in. For
example:

        hg branch <yourname>_first_issue

1. Check that you're on the new branch with `hg branch` again:

        hg branch

1. Now this is the part of the tutorial you've been waiting for! Here's where you
will do your magic! Use the tools you were introduced to in this series to fix
the issue you chose to address.

1. If you get stuck at any time, you're welcome to ask questions on
[Gazebo Answers](http://answers.gazebosim.org/) or on the issue you're trying
to fix.

1. After you've made a few changes, be sure to "commit" your progress so it
doesn't get lost. You do this with the `hg commit` command, and pass it a
custom message, for example:

        hg commit -m "My first commit: started fixing a bug!"

1. Keep working on the issue and committing your progress until you're
satisfied with the result. Remember the quality guidelines from previous
tutorials and make sure your code is tested, documented and follows the style
guide.

1. Take some time to review everything line-by-line. The review process will go
much faster if you make sure everything is perfect before other people look at
your code.

1. Once you're done, push your commits to your online repository on Bitbucket
as follows:

        hg push --new

    > The `new` flag is needed because you created a new branch

    > By default, mercurial will push to the repository you cloned from

1. Fill in your username and password, then check on your Bitbucket repository
that the new branch is there.

        https://bitbucket.org/<yourname>/gazebo/branches

## Make a pull request

It's time to put your code through code review. You will request that the
Gazebo team pull your changes into the official repository through a
"pull request". Here is the list of pull requests currently open for Gazebo:

[https://bitbucket.org/osrf/gazebo/pull-requests](https://bitbucket.org/osrf/gazebo/pull-requests)

It's a good idea to take a look at a couple of [pull requests which have
been accepted](https://bitbucket.org/osrf/gazebo/pull-requests/?state=MERGED)
in the past so you get an idea of how the process works.

You can create a pull request for Gazebo on this link:

[https://bitbucket.org/osrf/gazebo/pull-requests/new](https://bitbucket.org/osrf/gazebo/pull-requests/new)

Be sure to describe what you did, refer the issue you're fixing, and add any
images or other things which you think may help the reviewers get a good
idea of your code. Let us know in the description if this is your first
pull request so we can give you extra guidance ;)

At least two other people have to approve your pull request before it can be merged.
Be responsive to any questions and comments and adapt your code accordingly.

Once it is merged, you're done! You've contributed to Gazebo and your code will
be part of the next release! Congratulations!

