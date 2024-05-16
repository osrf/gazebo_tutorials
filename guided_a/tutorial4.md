# Code quality

Congratulations, by this point you're familiar with Gazebo's codebase and
are able to make changes to it! Before you dive in and start making a lot
of changes to Gazebo's source code, we should talk about code quality.
This tutorial will explain how to document and test your code, and comply
with Gazebo's coding style.

## Testing

Tests make sure that changes to the code are not breaking any existing
features. If you're making changes to Gazebo, you should make sure it doesn't
break any existing tests. If you're adding features, you should add tests
to make sure other people don't break them in the future.

Gazebo-classic uses [GTest](http://code.google.com/p/googletest) for general testing
and [QTest](http://doc.qt.io/qt-5/qtest.html) for GUI tests. There are a few
kinds of tests:

1. **Unit tests**: all classes should have corresponding unit tests. These live
in the same directory as the source code and are sufixed by `_TEST`. If you
add a new class, or new functions to a class, make sure to add a corresponding
unit test.

1. **Integration tests**: tests which verify how several classes are working
together go under the `tests/integration` directory. When adding features which span
multiple classes, be sure to add an integration test for it. Before creating a
new integration test file, check the current test files. If one closely matches
the topic of your new code, simply add a new test function to the file. Otherwise,
create a new test file, and write your test.

1. **Regression tests**: tests which fix broken features go under `tests/regression`
and are prefixed by the issue number on Gazebo's
[issue tracker](https://github.com/osrf/gazebo/issues).

If you need any help writing tests, feel free to ask a question on
[Gazebo Answers](http://answers.gazebosim.org).

### Build tests

It's possible to build each test individually. When you run `cmake` for the
first time, build targets are generated for each test. They are prefixed with
`UNIT_`, `INTEGRATION_` or `REGRESSION_`.

On the previous tutorial, we made changes to `gazebo/gui/TimeWidget.cc`. Let's
build the unit test for that class to make sure we didn't break anything:

1. Go to the build folder:

        cd ~/code/gazebo/build

1. Build the specific test you want to check. _Tip: you can press tab to
autocomplete the name of the test_:

        make UNIT_TimeWidget_TEST

1. The test will be built under the build folder, but following the path of its
source file. You can run it as follows from the build folder:

        ./gazebo/gui/UNIT_TimeWidget_TEST

1. Check that all tests passed.

It's usually enough to only run the tests which you foresee could be affected
by your changes. But in case you want to run all tests on Gazebo, you can do
the following, but beware that this may take several hours:

    make tests
    make test

### Test coverage

The goal is to achieve 100% line and branch coverage. However, this is not
always possible due to complexity and time constraints. Try to write as
complete of a test suite as possible, and use the coverage analysis tools as
guide.

Gazebo-classic has a build target called `make coverage` that will produce a code
coverage report. Here are the steps to run coverage:

1. Install [lcov](http://ltp.sourceforge.net/coverage/lcov.php):

        sudo apt install lcov

1. In your `build` folder, compile Gazebo-classic with `-DCMAKE_BUILD_TYPE=coverage`

        cd ~/code/gazebo/build
        cmake -DCMAKE_BUILD_TYPE=coverage ..
        make -j4

1. Run a single test, or all the tests, for example:

        make test

1. Make the coverage report

        make coverage

1. View the coverage report on a browser

        firefox coverage/index.html

### Automated testing

Tests are very important to catch problems which us humans might let slip
through. That's why we don't rely solely on running tests locally, but
also have automated tools in place to run all tests on all supported
platforms (Ubuntu, MacOSX and Windows) before fully integrating new code into
Gazebo.

More detail is coming on the next tutorials, but rest assured that if you
happen to miss a test failure locally, there will be other checks in place to
catch it later.

## Warnings

Be careful not to introduce any compiler warnings to the code. As you're
building Gazebo, keep an eye out for warnings and be sure to fix them whenever
they come up.

## Style

Static code checking analyzes your code for bugs, such as potential memory
leaks, and style. Following the style is important to maintain a common look
and feel across the whle codebase, making it easier for several contributors to
work together.

Gazebo's static code checker uses `cppcheck`, and a modified `cpplint`. To run
the style checker:

1. Make sure you have cppcheck installed:

        sudo apt-get install cppcheck

1. Run the following script from the root of the Gazebo-classic sources:

        cd ~/code/gazebo
        sh tools/code_check.sh

1. It takes a few minutes to run. Fix all errors and warnings until the output
looks like:

        Total errors found: 0

The tool does not catch all style errors. Be sure to take a look at Gazebo's
[style guide](/tutorials?tut=contrib_code&cat=development#Style)
and follow all the directions contained there too.

## Documentation

Documenting your code is important to help others understand what the code is
doing, that includes your future self.

Document all your code. Every class, function, member variable must have
[doxygen](http://www.stack.nl/~dimitri/doxygen/)
comments. All code in source files must have documentation that describes the
functionality.

