# Introduction

Interested in developing new features, fixing bugs, or contributing code you
have laying around? Read on to find out how.

## Development process

We follow a development process designed to reduce errors, encourage
collaboration, and make high quality code. The process may seem rigid and
tedious, but every step is worth the effort (especially if you like
applications that work). 

### Steps to follow

We will use the Gazebo repository as an example, but the step apply equally
well to other repositories maintained by OSRF.

1. Are you sure?
> Run through this mental checklist before getting started.
>
> 1. Has your idea already been done, or maybe someone is already working on it?
>
>     Check [answers.gazebosim.org](http://answers.gazebosim.org) and the [issue tracker](https://bitbucket.org/osrf/gazebo/issues).
>
> 1. Get feedback from the Gazebo core team.
>     Send an email to the [mailing list](https://groups.google.com/a/osrfoundation.org/forum/#!forum/gazebo), post a question on [answers.gazebosim.org](http://answers.gazebosim.org), or use the [issue tracker](https://bitbucket.org/osrf/gazebo/issues) to get feedback from Gazebo developers.

1. [Fork Gazebo](https://bitbucket.org/osrf/gazebo/fork)
> This will create your own personal copy of Gazebo. All of your development should take place in your fork.

1. Work out of a branch: `hg branch my_new_branch_name`
> Always work out of a new branch, never off of default. This is a good habit to get in, and will make your life easier.

1. Write your code.
> This is the fun part.

1. Write tests.
> A pull request will only be accepted if it has tests.

1. Compiler warnings. 
>  Code must have zero compile warnings. This currently only applies to Linux.

1. Style
> A tool is provided in Gazebo (and other repositories) to check for correct style. Your code must have no errors after running the following command from the root of the source tree:
>
> `sh tools/code_check.sh`
>
> The tool does not catch all style errors. See the Style section below for more information.

1. Tests pass
> There must be no failing tests. You can check by running `make test` in your build directory.

1. Documentation.
> Document all your code. Every class, function, member variable must have doxygen comments. All code in source files must have documentation that describes the functionality. This will help reviewers, and future developers.

1. Review your code.
> Before submitting your code through a pull request, take some time to review everything line-by-line. The review process will go much faster if you make sure everything is perfect before other people look at your code. There is a bit of the human-condition involved here. Folks are less likely to spend time reviewing your code if it's bad.

1. Small pull requests
> A large pull request is hard to review, and will take a long time. It is worth your time to split a large pull request into multiple smaller pull requests. For reference, here are a few examples:
>
> [Small, very nice](https://bitbucket.org/osrf/gazebo/pull-request/1732)
>
> [Medium, still okay](https://bitbucket.org/osrf/gazebo/pull-request/1700/)
>
> [Too large](https://bitbucket.org/osrf/gazebo/pull-request/30)

1. [Pull request](https://bitbucket.org/osrf/gazebo/pull-request/new)
> Submit a pull request when you ready.

1. Review
> At least two other people have to approve your pull request before it can be merged. Please be responsive to any questions and comments.

1. [Dashboard](http://gazebosim.org/dashboard)
> We have a dashboard that lists all open pull requests. Only pull requests above the green line can be merged, and they must also have at least two approvals.
> We do this to encourage the review of pull requests. If you submit a pull request, then try to review other open pull requests. This will reduce the time it takes for your code to get accepted, and also gets more eyes on more code.

1. Done, phew.
> Once you have met all the requirements, you're code will be merged. Thanks for improving Gazebo!


# Style

In general, we follow [Google's style guide](https://google-styleguide.googlecode.com/svn/trunk/cppguide.html). However, we add in some extras.

1. **This pointer**
> All class attributes and member functions must be accessed using the `this->` pointer. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Base.cc#cl-40).

1. **Underscore function parameters**
> All function parameters must start with an underscore. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Base.cc#cl-77).

1. **Do not cuddle braces**
> All braces must be on their own line. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Base.cc#cl-131).

1. **Multi-line code blocks**
> If a block of code spans multiple lines and is part of a flow control statement, such as an `if`, then it must be wrapped in braces. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Base.cc#cl-249)

1. **++ operator**
> This occurs mostly in `for` loops. Prefix the `++` operator, which is [slightly more effecient than postfix in some cases](http://programmers.stackexchange.com/questions/59880/avoid-postfix-increment-operator).

1. **PIMPL/Opaque pointer**
> If you are writing a new class, it must use a private data pointer. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/World.hh?at=default#cl-479), and you can read more [here](https://en.wikipedia.org/wiki/Opaque_pointer).  


# Reduce Code Duplication 

Check to make sure someone else is not currently working on the same feature, before embarking on a project to add something to Gazebo. Simply send a quick email to the Gazebo mailing list expressing your interest and idea. Someone will get back to you shortly about your idea.
  
# Write Tests

All code should have a corresponding unit test. Gazebo uses [http://code.google.com/p/googletest/ GTest] for unit testing. All regression test should be placed in `<gazebo_sources>/test/regresssion/`. 

Before creating a new regressions test file, check the current test files. If one closely matches the topic of your new code, simply add a new test function to the file. Otherwise, create a new test file, and write your test. 

## Gazebo assertions

### What is an assertion?

An assertion is a check, which always produce a boolean result, that developers place in the code when want to be sure that check is always true. They are aimed to detect programming errors and should check for impossible situations in the code. If the assertion check failed, the assertion will stop the program immediately.

     Object * p = some_crazy_function()
     GZ_ASSERT(p != NULL, “Object from some_crazy_function should never point to NULL”)
     p->run()

### Gazebo runtime assertions: GZ_ASSERT

In Gazebo, the GZ_ASSERT macro id designed to handle all our runtime assertions

     GZ_ASSERT(<condition to check>,<fail msg>) 

* '''condition-to-check:''' anything returning a boolean value that should always be true.
* '''fail msg:''' message displaied when assertion is thrown

### Benefits of the assertions

Some of the beneficts of using the assertions:
* They are really useful for not having to debug all kind of weird and unexpected errors, especially in runtime. Exact failure point appears when pass by an assertion.
* Developer can be sure that some conditions are met at a given code point. Code turns more reliable.
* Help to detect no so obvious errors happening (affecting performance for example)

### Difference between Assertion and Exception

While assertions are aimed to impossible situations generated from programming errors, the exceptions handle all kind of expected errors and unusual but logically possible code situations.

Lets review an example: imaging we are writing a math library and created a really fast method to calculate square roots but it only works for positive numbers. Something declared as:
     
     double sqrt_for_positives(double number)

So what could be an assertion and what an exception for our revolutionary function?

* Exception: if the incoming number is negative (our function only accept positive numbers), then we will thrown an exception. It was an error by the user but we should consider it a possible scenario since we are offering a public interface.

* Assertion: our square root should never return a negative number. This is not a logical error, is a completely unexpected error.

## Debugging Gazebo

### Meaningful backtraces

In order to provide meaningful backtraces when using a debugger, such as GDB, Gazebo should be compiled with debugging support enabled. When using the ubuntu packages, specially the ''-dbg'' package, this support is limited but could be enough in most of the situations. This are the three level of traces which can be obtained:

'''Maximum level of debugging support'''
:This only can be obtained compiling Gazebo from source and setting the `CMAKE_BUILD_TYPE` to `DEBUG`. This will set up no optimizations and debugging symbols. It can be required by developers in situations specially difficult to reproduce.

'''Medium level of debugging support'''
:This can be obtained installing the ''gazebo-dbg'' package (since 1.4 version) or compiling Gazebo from source using the `RELWITHDEBINFO` `CMAKE_BUILD_TYPE` mode (which is the default if no mode is provided). This will set up ''-O2'' optimization level but provide debugging symbols. This should be the default when firing up gdb to explore errors and submit traces.

'''Minimum level of debugging support'''
:This one is present in package versions previous to 1.4 (no ''-dbg'' package present) or compiling Gazebo from source using the `RELEASE` `CMAKE_BUILD_TYPE` option. This will set up the maximum level of optimizations and does not provide any debugging symbol information. This traces are particularly difficult to follow.

## Code Check 

Code pushed into the Gazebo repository should pass a few simple tests. It is also helpful if patches submitted through bitbucket pass these tests. Passing these tests is defined as generating no error or warning messages for each of the following tests.

### Regression Tests   

In your Gazebo build directory run `make test`:
        make test

All the tests should pass. If they do not, you can run and debug the tests individually. For example, to run the transport test from your build directory:

        ./test/regression/transport

### Static Code Check

Static code checking analyzes your code for bugs, such as potential memory leaks, and style. The Gazebo static code checker uses cppcheck, and a modified cpplint. You'll need to install cppcheck on your system. Ubuntu users can install via:

        sudo apt-get install cppcheck

To check your code, run the following script from the root of the Gazebo sources:

        sh tools/code_check.sh

It takes a few minutes to run. Fix all errors and warnings until the output looks like:

        Total errors found: 0

### `CMAKE_BUILD_TYPE`=Check compiles with no warnings

This test compiles Gazebo with numerous warning flags enabled. The source code for Gazebo should compile cleanly. This does not include code in the {{{deps}}} directory. As a rule of thumb, start looking for compilation warnings after the proto messages are built which appear as a series of blue text:

        Linking CXX executable gazebomsgs_out
        Running C++ protocol buffer compiler on axis.proto
        Running C++ protocol buffer compiler on boxgeom.proto
        Running C++ protocol buffer compiler on camerasensor.proto
        Running C++ protocol buffer compiler on collision.proto
        Running C++ protocol buffer compiler on color.proto
        Running C++ protocol buffer compiler on contact.proto
        Running C++ protocol buffer compiler on contacts.proto

