# Introduction

This page details how developers should go about creating and contributing code to Gazebo.

# Reduce Code Duplication 

Check to make sure someone else is not currently working on the same feature, before embarking on a project to add something to Gazebo. Simply send a quick email to the Gazebo mailing list expressing your interest and idea. Someone will get back to you shortly about your idea.
  
# Write Tests

All code should have a corresponding unit test. Gazebo uses [http://code.google.com/p/googletest/ GTest] for unit testing. All regression test should be placed in `<gazebo_sources>/test/regresssion/`. 

Before creating a new regressions test file, check the current test files. If one closely matches the topic of your new code, simply add a new test function to the file. Otherwise, create a new test file, and write your test. 

## Gazebo assertions

### What is an assertion?
An assertion is a check which produces a boolean result of true or false. Developers place them in code when they want to be sure that an assumption they have made is true.
They are aimed at detecting programming errors and should check for impossible situations in the code. If the assertion check failed, the assertion will stop the program immediately.

     Object * p = some_crazy_function()
     GZ_ASSERT(p != NULL, “Object from some_crazy_function should never point to NULL”)
     p->run()

### Gazebo runtime assertions: GZ_ASSERT

In Gazebo, the GZ_ASSERT macro id designed to handle all our runtime assertions

     GZ_ASSERT(<condition to check>,<fail msg>) 

* '''condition-to-check:''' anything returning a boolean value that should always be true.
* '''fail msg:''' message displaied when assertion is thrown

### Benefits of assertions

Some of the benefits of using assertions:
* They are really useful for not having to debug all kind of weird and unexpected errors, especially during runtime. Exact failure point appears when pass by an assertion.
* Developer can be sure that some conditions are met at a given code point. Code becomes more reliable.
* They help to detect not-so-obvious errors happening (affecting performance for example)

### Difference between Assertion and Exception

While assertions are aimed at impossible situations generated from programming errors, exceptions handle all kinds of expected errors and unusual but logically possible code situations.

Let's review an example: imagine we are writing a math library and created a really fast method to calculate square roots but it only works for positive numbers. Something declared as:
     
     double sqrt_for_positives(double number)

So what might be an assertion or exception for our revolutionary function?

* Exception: if the incoming number is negative (our function only accept positive numbers), then we will thrown an exception. It was an error by the user but we should consider it a possible scenario since we are offering a public interface.

* Assertion: our square root should never return a negative number. This is not a logical error, is a completely unexpected error.

## Debugging Gazebo

### Meaningful backtraces

In order to provide meaningful backtraces when using a debugger, such as GDB, Gazebo should be compiled with debugging support enabled. When using the ubuntu packages, specially the ''-dbg'' package, this support is limited but could be enough in most situations. These are the three levels of traces which can be obtained:

'''Maximum level of debugging support'''
:This only can be obtained by compiling Gazebo from source and setting the `CMAKE_BUILD_TYPE` to `DEBUG`. This will set up no optimizations and debugging symbols. It can be required by developers in situations especially difficult to reproduce.

'''Medium level of debugging support'''
:This can be obtained by installing the ''gazebo-dbg'' package (since 1.4 version) or compiling Gazebo from source using the `RELWITHDEBINFO` `CMAKE_BUILD_TYPE` mode (which is the default if no mode is provided). This will set up ''-O2'' optimization level but provide debugging symbols. This should be the default when firing up gdb to explore errors and submit traces.

'''Minimum level of debugging support'''
:This one is present in package versions previous to 1.4 (no ''-dbg'' package present) or compiling Gazebo from source using the `RELEASE` `CMAKE_BUILD_TYPE` option. This will set up the maximum level of optimizations and does not provide any debugging symbol information. These traces are particularly difficult to follow.

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

