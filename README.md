# React [![License](http://img.shields.io/badge/license-MIT-lightgrey.svg?style=flat)][License] [![Flattr this](http://img.shields.io/badge/flattr-donate-lightgrey.svg?style=flat)][Donate] [![Build Status](http://img.shields.io/travis/flow/react/develop.svg?style=flat)](https://travis-ci.org/flow/react) [![Coverage Status](http://img.shields.io/coveralls/flow/react/develop.svg?style=flat)](https://coveralls.io/r/flow/react)

Real-time 3D physics library for Java, based on the [ReactPhysics3D](https://code.google.com/p/reactphysics3d/) C++ library by [Daniel Chappuis](http://www.danielchappuis.ch/). 

## Features
* Rigid body dynamics
* Discrete collision detection
* Collision shapes (Sphere, Box, Cone, Cylinder, Capsule, Convex Mesh)
* Broadphase collision detection (Sweep and Prune using AABB)
* Narrowphase collision detection (GJK/EPA)
* Collision response and friction (Sequential Impulses solver)
* Joints (Ball-and-socket, Hinge, Slider, Fixed)
* Sleeping technique for inactive bodies
* Multi-platform (Windows, Linux, Mac OS X)
* Unit tests and Javadocs

## Getting Started
* [Examples and code snippets](https://github.com/flow/examples/tree/master/react)
* [Official documentation](#documentation)
* [IRC support chat](http://kiwiirc.com/client/irc.esper.net/flow)
* [Issues tracker](https://github.com/flow/react/issues)

## Source Code
The latest and greatest source can be found here on [GitHub](https://github.com/flow/react). If you are using Git, use this command to clone the project:

    git clone git://github.com/flow/react.git

Or download the latest [development archive](https://github.com/flow/react/archive/develop.zip) or the latest [stable archive](https://github.com/flow/react/archive/master.zip).

## Test Dependencies
The following dependencies are only needed if you compiling the tests included with this project. Gotta test 'em all!
* [junit:junit](https://oss.sonatype.org/#nexus-search;gav~junit~junit~~~)

## Building from Source
This project can be built with the _latest_ [Java Development Kit](http://oracle.com/technetwork/java/javase/downloads) and [Maven](https://maven.apache.org/) or [Gradle](https://www.gradle.org/). Maven and Gradle are used to simplify dependency management, but using either of them is optional.

For Maven, the command `mvn clean package` will build the project and will put the compiled JAR in `target`, and `mvn clean install` will copy it to your local Maven repository.

For Gradle, the command `gradlew` will build the project and will put the compiled JAR in `~/build/distributions`, and `gradlew install` will copy it to your local Maven repository.

## Contributing
Are you a talented programmer looking to contribute some code? We'd love the help!

* Open a pull request with your changes, following our [guidelines and coding standards](CONTRIBUTING.md).
* Please follow the above guidelines for your pull request(s) accepted.
* For help setting up the project, keep reading!

Love the project? Feel free to [donate] to help continue development! Flow projects are open-source and powered by community members, like yourself. Without you, we wouldn't be here today!

Don't forget to watch and star our repo to keep up-to-date with the latest Flow development!

## Usage
If you're using [Maven](https://maven.apache.org/download.html) to manage project dependencies, simply include the following in your `pom.xml` file:

    <dependency>
        <groupId>com.flowpowered</groupId>
        <artifactId>react</artifactId>
        <version>1.0.1-SNAPSHOT</version>
    </dependency>

If you're using [Gradle](https://www.gradle.org/) to manage project dependencies, simply include the following in your `build.gradle` file:

    repositories {
        mavenCentral()
    }
    dependencies {
        compile 'com.flowpowered:react:1.0.1-SNAPSHOT'
    }

If you plan on using snapshots and do not already have the snapshot repo in your repository list, you will need to add this as well:

    https://oss.sonatype.org/content/groups/public/

If you'd prefer to manually import the latest .jar file, you can get it [here](https://github.com/flow/react/releases).

## Documentation
Want to get friendly with the project and put it to good use? Check out the latest [Javadocs](https://flowpowered.com/react).

To generate Javadocs with Maven, use the `mvn javadoc:javadoc` command. To view the Javadocs simply go to `target/site/apidocs/` and open `index.html` in a web browser.

To generate Javadocs with Gradle, use the `gradlew javadoc` command. To view the Javadocs simply go to `build/docs/javadoc/` and open `index.html` in a web browser.

## Version Control
We've adopted the [git flow branching model](http://nvie.com/posts/a-successful-git-branching-model/) in our projects. The creators of git flow released a [short intro video](http://vimeo.com/16018419) to explain the model.

The `master` branch is production-ready and contains the latest tagged releases. Before a release is made, it is stagged in `release/x` branches before being pushed and tagged in the `master` branch. Small patches from `hotfix/x` branches are also pushed to `master`, and will always have a release version. The `develop` branch is pre-production, and is where we push `feature/x` branches for testing.

## Legal Stuff
React is licensed under the [MIT License][License]. Basically, you can do whatever you want as long as you include the original copyright. Please see the `LICENSE.txt` file for details.

## Credits
* Daniel Chappuis and contributors of the original [ReactPhysics3d](https://code.google.com/p/reactphysics3d/) C++ library.
* [Spout](https://spout.org/) and contributors - *where we all began, and for much of the re-licensed code.*
* All the people behind [Java](http://www.oracle.com/technetwork/java/index.html), [Maven](https://maven.apache.org/), and [Gradle](https://www.gradle.org/).

[Donate]: https://flattr.com/submit/auto?user_id=spout&url=https://github.com/flow/react&title=React&language=Java&tags=github&category=software
[License]: https://tldrlegal.com/l/mit
