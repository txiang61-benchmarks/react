/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.flowpowered.react.engine;

import units.qual.Dimensionless;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;

import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.ReactDefaults.ContactsPositionCorrectionTechnique;
import com.flowpowered.react.Utilities.IntPair;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.collision.BroadPhasePair;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.constraint.BallAndSocketJoint;
import com.flowpowered.react.constraint.BallAndSocketJoint.BallAndSocketJointInfo;
import com.flowpowered.react.constraint.ConstraintSolver;
import com.flowpowered.react.constraint.ContactPoint;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.constraint.FixedJoint;
import com.flowpowered.react.constraint.FixedJoint.FixedJointInfo;
import com.flowpowered.react.constraint.HingeJoint;
import com.flowpowered.react.constraint.HingeJoint.HingeJointInfo;
import com.flowpowered.react.constraint.Joint;
import com.flowpowered.react.constraint.Joint.JointInfo;
import com.flowpowered.react.constraint.Joint.JointListElement;
import com.flowpowered.react.constraint.SliderJoint;
import com.flowpowered.react.constraint.SliderJoint.SliderJointInfo;
import com.flowpowered.react.engine.ContactManifold.ContactManifoldListElement;
import com.flowpowered.react.math.Mathematics;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Quaternion;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * This class represents a dynamics world. This class inherits from the CollisionWorld class. In a dynamics world bodies can collide and their movements are simulated using the laws of physics.
 */
public class DynamicsWorld extends CollisionWorld {
    private final @Dimensionless Timer mTimer;
    private final @Dimensionless ContactSolver mContactSolver;
    private final @Dimensionless ConstraintSolver mConstraintSolver;
    private @Dimensionless int mNbVelocitySolverIterations;
    private @Dimensionless int mNbPositionSolverIterations;
    private @Dimensionless boolean mIsSleepingEnabled;
    private final @Dimensionless Set<@Dimensionless RigidBody> mRigidBodies = new @Dimensionless HashSet<>();
    private final @Dimensionless List<@Dimensionless ContactManifold> mContactManifolds = new @Dimensionless ArrayList<>();
    private final @Dimensionless Set<@Dimensionless Joint> mJoints = new @Dimensionless HashSet<>();
    private final @Dimensionless Vector3 mGravity;
    private @Dimensionless boolean mIsGravityEnabled;
    private @Dimensionless Vector3 @Dimensionless [] mConstrainedLinearVelocities;
    private @Dimensionless Vector3 @Dimensionless [] mConstrainedAngularVelocities;
    private @Dimensionless Vector3 @Dimensionless [] mSplitLinearVelocities;
    private @Dimensionless Vector3 @Dimensionless [] mSplitAngularVelocities;
    private final @Dimensionless ArrayList<@Dimensionless Vector3> mConstrainedPositions = new @Dimensionless ArrayList<>();
    private final @Dimensionless ArrayList<@Dimensionless Quaternion> mConstrainedOrientations = new @Dimensionless ArrayList<>();
    private final @Dimensionless TObjectIntMap<@Dimensionless RigidBody> mMapBodyToConstrainedVelocityIndex = new @Dimensionless TObjectIntHashMap<>();
    private @Dimensionless int mNbIslands;
    private @Dimensionless int mNbIslandsCapacity;
    private @Dimensionless Island @Dimensionless [] mIslands;
    private @Dimensionless int mNbBodiesCapacity;
    private @Dimensionless float mSleepLinearVelocity;
    private @Dimensionless float mSleepAngularVelocity;
    private @Dimensionless float mTimeBeforeSleep;
    private @Dimensionless EventListener mEventListener;
    private @Dimensionless boolean isTicking = false;
    // Tick cache
    private final @Dimensionless Set<@Dimensionless RigidBody> mRigidBodiesToAddCache = new @Dimensionless HashSet<>();
    private final @Dimensionless Set<@Dimensionless RigidBody> mRigidBodiesToDeleteCache = new @Dimensionless HashSet<>();

    /**
     * Constructs a new dynamics world from the gravity and the default time step.
     *
     * @param gravity The gravity
     */
    public DynamicsWorld(Vector3 gravity) {
        this(gravity, ReactDefaults.DEFAULT_TIMESTEP);
    }

    /**
     * Constructs a new dynamics world from the gravity and the time step.
     *
     * @param gravity The gravity
     * @param timeStep The time step
     */
    public DynamicsWorld(Vector3 gravity, float timeStep) {
        mTimer = new @Dimensionless Timer(timeStep);
        mGravity = gravity;
        mIsGravityEnabled = true;
        @Dimensionless
        Vector3 @Dimensionless [] empty = new Vector3 @Dimensionless [((@Dimensionless int) (0))];
        mConstrainedLinearVelocities = empty;
        mConstrainedAngularVelocities = empty;
        mContactSolver = new @Dimensionless ContactSolver(mMapBodyToConstrainedVelocityIndex);
        mConstraintSolver = new @Dimensionless ConstraintSolver(mConstrainedPositions, mConstrainedOrientations, mMapBodyToConstrainedVelocityIndex);
        mNbVelocitySolverIterations = ReactDefaults.DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS;
        mNbPositionSolverIterations = ReactDefaults.DEFAULT_POSITION_SOLVER_NB_ITERATIONS;
        mIsSleepingEnabled = ReactDefaults.SLEEPING_ENABLED;
        mSplitLinearVelocities = empty;
        mSplitAngularVelocities = empty;
        mNbIslands = ((@Dimensionless int) (0));
        mNbIslandsCapacity = ((@Dimensionless int) (0));
        mIslands = null;
        mNbBodiesCapacity = ((@Dimensionless int) (0));
        mSleepLinearVelocity = ReactDefaults.DEFAULT_SLEEP_LINEAR_VELOCITY;
        mSleepAngularVelocity = ReactDefaults.DEFAULT_SLEEP_ANGULAR_VELOCITY;
        mTimeBeforeSleep = ReactDefaults.DEFAULT_TIME_BEFORE_SLEEP;
    }

    /**
     * Starts the physics simulation.
     */
    public void start(@Dimensionless DynamicsWorld this) {
        mTimer.start();
    }

    /**
     * Stops the physics simulation.
     */
    public void stop(@Dimensionless DynamicsWorld this) {
        mTimer.stop();
    }

    /**
     * Sets the number of iterations for the velocity constraint solver.
     *
     * @param nbIterations The number of iterations to do
     */
    public void setNbIterationsVelocitySolver(@Dimensionless DynamicsWorld this, @Dimensionless int nbIterations) {
        mNbVelocitySolverIterations = nbIterations;
    }

    /**
     * Sets the number of iterations for the position constraint solver.
     *
     * @param nbIterations The number of iterations to do
     */
    public void setNbIterationsPositionSolver(@Dimensionless DynamicsWorld this, @Dimensionless int nbIterations) {
        mNbPositionSolverIterations = nbIterations;
    }

    /**
     * Sets the position correction technique used for contacts.
     *
     * @param technique The technique to use
     */
    public void setContactsPositionCorrectionTechnique(@Dimensionless DynamicsWorld this, @Dimensionless ContactsPositionCorrectionTechnique technique) {
        if (technique == ContactsPositionCorrectionTechnique.BAUMGARTE_CONTACTS) {
            mContactSolver.setIsSplitImpulseActive(false);
        } else {
            mContactSolver.setIsSplitImpulseActive(true);
        }
    }

    /**
     * Activates or deactivates the solving of friction constraints at the center of the contact manifold instead of solving them at each contact point.
     *
     * @param isActive Whether or not to solve the friction constraint at the center of the manifold
     */
    public void setSolveFrictionAtContactManifoldCenterActive(@Dimensionless DynamicsWorld this, @Dimensionless boolean isActive) {
        mContactSolver.setSolveFrictionAtContactManifoldCenterActive(isActive);
    }

    /**
     * Returns the gravity vector for the world.
     *
     * @return The gravity vector
     */
    public @Dimensionless Vector3 getGravity(@Dimensionless DynamicsWorld this) {
        return mGravity;
    }

    /**
     * Returns true if the gravity is on.
     *
     * @return Whether or not the gravity is on
     */
    public @Dimensionless boolean isGravityEnabled(@Dimensionless DynamicsWorld this) {
        return mIsGravityEnabled;
    }

    /**
     * Sets the gravity on if true, off is false.
     *
     * @param isGravityEnabled True to turn on the gravity, false to turn it off
     */
    public void setIsGravityEnabled(@Dimensionless DynamicsWorld this, @Dimensionless boolean isGravityEnabled) {
        mIsGravityEnabled = isGravityEnabled;
    }

    /**
     * Returns true if the sleeping technique is enabled.
     *
     * @return Whether or not sleeping is enabled
     */
    public @Dimensionless boolean isSleepingEnabled(@Dimensionless DynamicsWorld this) {
        return mIsSleepingEnabled;
    }

    /**
     * Returns the current sleep linear velocity.
     *
     * @return The sleep linear velocity
     */
    public @Dimensionless float getSleepLinearVelocity(@Dimensionless DynamicsWorld this) {
        return mSleepLinearVelocity;
    }

    /**
     * Sets the sleep linear velocity. When the velocity of a body becomes smaller than the sleep linear/angular velocity for a given amount of time, the body starts sleeping and does not need to be
     * simulated anymore.
     *
     * @param sleepLinearVelocity The sleep linear velocity
     */
    public void setSleepLinearVelocity(@Dimensionless DynamicsWorld this, @Dimensionless float sleepLinearVelocity) {
        if (sleepLinearVelocity < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Sleep linear angular velocity must be greater or equal to zero");
        }
        mSleepLinearVelocity = sleepLinearVelocity;
    }

    /**
     * Returns the current sleep angular velocity.
     *
     * @return The sleep angular velocity
     */
    public @Dimensionless float getSleepAngularVelocity(@Dimensionless DynamicsWorld this) {
        return mSleepAngularVelocity;
    }

    /**
     * Sets the sleep angular velocity. When the velocity of a body becomes smaller than the sleep linear/angular velocity for a given amount of time, the body starts sleeping and does not need  to be
     * simulated anymore.
     *
     * @param sleepAngularVelocity The sleep angular velocity
     */
    public void setSleepAngularVelocity(@Dimensionless DynamicsWorld this, @Dimensionless float sleepAngularVelocity) {
        if (sleepAngularVelocity < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Sleep angular velocity must be greater or equal to zero");
        }
        mSleepAngularVelocity = sleepAngularVelocity;
    }

    /**
     * Returns the time a body is required to stay still before sleeping.
     *
     * @return The time before sleep
     */
    public @Dimensionless float getTimeBeforeSleep(@Dimensionless DynamicsWorld this) {
        return mTimeBeforeSleep;
    }

    /**
     * Sets the time a body is required to stay still before sleeping.
     *
     * @param timeBeforeSleep The time before sleep
     */
    public void setTimeBeforeSleep(@Dimensionless DynamicsWorld this, @Dimensionless float timeBeforeSleep) {
        if (timeBeforeSleep < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Time before sleep must be greater or equal to zero");
        }
        mTimeBeforeSleep = timeBeforeSleep;
    }

    /**
     * Sets an event listener object to receive events callbacks. If you use <code>null</code> as an argument, the events callbacks will be disabled.
     *
     * @param eventListener The event listener, or null for none
     */
    public void setEventListener(@Dimensionless DynamicsWorld this, @Dimensionless EventListener eventListener) {
        mEventListener = eventListener;
    }

    /**
     * Gets the number of rigid bodies in the world.
     *
     * @return The number of rigid bodies in the world
     */
    public @Dimensionless int getNbRigidBodies(@Dimensionless DynamicsWorld this) {
        return mRigidBodies.size();
    }

    /**
     * Returns the number of joints in the world.
     *
     * @return The number of joints
     */
    public @Dimensionless int getNbJoints(@Dimensionless DynamicsWorld this) {
        return mJoints.size();
    }

    /**
     * Gets the set of bodies of the physics world.
     *
     * @return The rigid bodies
     */
    public @Dimensionless Set<@Dimensionless RigidBody> getRigidBodies(@Dimensionless DynamicsWorld this) {
        return mRigidBodies;
    }

    /**
     * Returns a reference to the contact manifolds of the world.
     *
     * @return The contact manifolds
     */
    public @Dimensionless List<@Dimensionless ContactManifold> getContactManifolds(@Dimensionless DynamicsWorld this) {
        return mContactManifolds;
    }

    /**
     * Gets the number of contact manifolds in the world.
     *
     * @return The number of contact manifolds in the world
     */
    public @Dimensionless int getNbContactManifolds(@Dimensionless DynamicsWorld this) {
        return mContactManifolds.size();
    }

    /**
     * Returns the current physics time (in seconds)
     *
     * @return The current physics time
     */
    public @Dimensionless double getPhysicsTime(@Dimensionless DynamicsWorld this) {
        return mTimer.getPhysicsTime();
    }

    /**
     * Updates the physics simulation. The elapsed time is determined by the timer. A step is only actually taken if enough time has passed. If a lot of time has passed, more than twice the time step,
     * multiple steps will be taken, to catch up.
     */
    public void update(@Dimensionless DynamicsWorld this) {
        if (!mTimer.isRunning()) {
            throw new @Dimensionless IllegalStateException("timer must be running");
        }
        mTimer.update();
        isTicking = true;
        while (mTimer.isPossibleToTakeStep()) {
            tick();
        }
        isTicking = false;
        resetBodiesForceAndTorque();
        setInterpolationFactorToAllBodies();
        disperseCache();
    }

    // LinkedDynamicsWorld needs to clearLinkedBodies at the end of a tick, not update
    protected void tick(@Dimensionless DynamicsWorld this) {
        mContactManifolds.clear();
        resetContactManifoldListsOfBodies();
        mCollisionDetection.computeCollisionDetection();
        computeIslands();
        integrateRigidBodiesVelocities();
        resetBodiesMovementVariable();
        mTimer.nextStep();
        solveContactsAndConstraints();
        integrateRigidBodiesPositions();
        solvePositionCorrection();
        if (mIsSleepingEnabled) {
            updateSleepingBodies();
        }
        updateRigidBodiesAABB();
    }

    // Resets the boolean movement variable for each body.
    private void resetBodiesMovementVariable(@Dimensionless DynamicsWorld this) {
        for (@Dimensionless RigidBody rigidBody : mRigidBodies) {
            rigidBody.setHasMoved(false);
        }
    }

    // Integrates the position and orientation of the rigid bodies using the provided time delta.
    // The positions and orientations of the bodies are integrated using the symplectic Euler time stepping scheme.
    private void integrateRigidBodiesPositions(@Dimensionless DynamicsWorld this) {
        final @Dimensionless float dt = (@Dimensionless float) mTimer.getTimeStep();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbIslands; i++) {
            final @Dimensionless RigidBody @Dimensionless [] bodies = mIslands[i].getBodies();
            for (@Dimensionless int b = ((@Dimensionless int) (0)); b < mIslands[i].getNbBodies(); b++) {
                if (bodies[b].isMotionEnabled()) {
                    final @Dimensionless int indexArray = mMapBodyToConstrainedVelocityIndex.get(bodies[b]);
                    final @Dimensionless Vector3 newLinVelocity = mConstrainedLinearVelocities[indexArray];
                    final @Dimensionless Vector3 newAngVelocity = mConstrainedAngularVelocities[indexArray];
                    bodies[b].setLinearVelocity(newLinVelocity);
                    bodies[b].setAngularVelocity(newAngVelocity);
                    if (mContactSolver.isSplitImpulseActive()) {
                        newLinVelocity.add(mSplitLinearVelocities[indexArray]);
                        newAngVelocity.add(mSplitAngularVelocities[indexArray]);
                    }
                    final @Dimensionless Vector3 currentPosition = bodies[b].getTransform().getPosition();
                    final @Dimensionless Quaternion currentOrientation = bodies[b].getTransform().getOrientation();
                    final @Dimensionless Vector3 newPosition = Vector3.add(currentPosition, Vector3.multiply(newLinVelocity, dt));
                    final @Dimensionless Quaternion newOrientation = Quaternion.add(currentOrientation, Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), newAngVelocity), currentOrientation), ((@Dimensionless float) (0.5f)) * dt));
                    final @Dimensionless Transform newTransform = new @Dimensionless Transform(newPosition, newOrientation.getUnit());
                    bodies[b].setTransform(newTransform);
                }
            }
        }
    }

    // Updates the AABBs of the bodies
    private void updateRigidBodiesAABB(@Dimensionless DynamicsWorld this) {
        for (@Dimensionless RigidBody rigidBody : mRigidBodies) {
            if (rigidBody.getHasMoved()) {
                rigidBody.updateAABB();
            }
        }
    }

    // Computes and set the interpolation factor for all bodies.
    private void setInterpolationFactorToAllBodies(@Dimensionless DynamicsWorld this) {
        final @Dimensionless float factor = mTimer.computeInterpolationFactor();
        if (factor < ((@Dimensionless int) (0)) && factor > ((@Dimensionless int) (1))) {
            throw new @Dimensionless IllegalStateException("interpolation factor must be greater or equal to zero"
                    + " and smaller or equal to one");
        }
        for (@Dimensionless RigidBody rigidBody : mRigidBodies) {
            rigidBody.setInterpolationFactor(factor);
        }
    }

    // Initialize the bodies velocities arrays for the next simulation step.
    private void initVelocityArrays() {
        final @Dimensionless int nbBodies = mRigidBodies.size();
        if (mNbBodiesCapacity != nbBodies && nbBodies > ((@Dimensionless int) (0))) {
            mNbBodiesCapacity = nbBodies;
            mSplitLinearVelocities = new @Dimensionless Vector3 @Dimensionless [mNbBodiesCapacity];
            mSplitAngularVelocities = new @Dimensionless Vector3 @Dimensionless [mNbBodiesCapacity];
            mConstrainedLinearVelocities = new @Dimensionless Vector3 @Dimensionless [mNbBodiesCapacity];
            mConstrainedAngularVelocities = new @Dimensionless Vector3 @Dimensionless [mNbBodiesCapacity];
        }
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbBodiesCapacity; i++) {
            mSplitLinearVelocities[i] = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
            mSplitAngularVelocities[i] = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        }
        mMapBodyToConstrainedVelocityIndex.clear();
        @Dimensionless
        int indexBody = ((@Dimensionless int) (0));
        for (@Dimensionless RigidBody rigidBody : mRigidBodies) {
            mMapBodyToConstrainedVelocityIndex.put(rigidBody, indexBody);
            indexBody++;
        }
    }

    // Integrates the constrained velocities array using the provided time delta.
    // This method only sets the temporary velocities and does not update
    // the actual velocities of the bodies. The velocities updated in this method
    // might violate the constraints and will be corrected in the constraint and
    // contact solver.
    private void integrateRigidBodiesVelocities(@Dimensionless DynamicsWorld this) {
        initVelocityArrays();
        final @Dimensionless float dt = (@Dimensionless float) mTimer.getTimeStep();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbIslands; i++) {
            final @Dimensionless RigidBody @Dimensionless [] bodies = mIslands[i].getBodies();
            for (@Dimensionless int b = ((@Dimensionless int) (0)); b < mIslands[i].getNbBodies(); b++) {
                @Dimensionless
                int indexBody = mMapBodyToConstrainedVelocityIndex.get(bodies[b]);
                if (bodies[b].isMotionEnabled()) {
                    mConstrainedLinearVelocities[indexBody] = Vector3.add(bodies[b].getLinearVelocity(), Vector3.multiply(dt * bodies[b].getMassInverse(), bodies[b].getExternalForce()));
                    mConstrainedAngularVelocities[indexBody] = Vector3.add(bodies[b].getAngularVelocity(),
                            Matrix3x3.multiply(Matrix3x3.multiply(dt, bodies[b].getInertiaTensorInverseWorld()), bodies[b].getExternalTorque()));
                    if (bodies[b].isGravityEnabled() && mIsGravityEnabled) {
                        mConstrainedLinearVelocities[indexBody].add(Vector3.multiply(dt * bodies[b].getMassInverse() * bodies[b].getMass(), mGravity));
                    }
                    final @Dimensionless float linDampingFactor = bodies[b].getLinearDamping();
                    final @Dimensionless float angDampingFactor = bodies[b].getAngularDamping();
                    final @Dimensionless float linearDamping = Mathematics.clamp(((@Dimensionless int) (1)) - dt * linDampingFactor, ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
                    final @Dimensionless float angularDamping = Mathematics.clamp(((@Dimensionless int) (1)) - dt * angDampingFactor, ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
                    mConstrainedLinearVelocities[indexBody].multiply(Mathematics.clamp(linearDamping, ((@Dimensionless int) (0)), ((@Dimensionless int) (1))));
                    mConstrainedAngularVelocities[indexBody].multiply(Mathematics.clamp(angularDamping, ((@Dimensionless int) (0)), ((@Dimensionless int) (1))));
                    bodies[b].updateOldTransform();
                } else {
                    mConstrainedLinearVelocities[indexBody] = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
                    mConstrainedAngularVelocities[indexBody] = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
                }
                indexBody++;
            }
        }
    }

    // Solves the contacts and constraints
    private void solveContactsAndConstraints(@Dimensionless DynamicsWorld this) {
        final @Dimensionless float dt = (@Dimensionless float) mTimer.getTimeStep();
        mContactSolver.setSplitVelocitiesArrays(mSplitLinearVelocities, mSplitAngularVelocities);
        mContactSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities, mConstrainedAngularVelocities);
        mConstraintSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities, mConstrainedAngularVelocities);
        for (@Dimensionless int islandIndex = ((@Dimensionless int) (0)); islandIndex < mNbIslands; islandIndex++) {
            final @Dimensionless boolean isConstraintsToSolve = mIslands[islandIndex].getNbJoints() > ((@Dimensionless int) (0));
            final @Dimensionless boolean isContactsToSolve = mIslands[islandIndex].getNbContactManifolds() > ((@Dimensionless int) (0));
            if (!isConstraintsToSolve && !isContactsToSolve) {
                continue;
            }
            if (isContactsToSolve) {
                mContactSolver.initializeForIsland(dt, mIslands[islandIndex]);
                mContactSolver.warmStart();
            }
            if (isConstraintsToSolve) {
                mConstraintSolver.initializeForIsland(dt, mIslands[islandIndex]);
            }
            for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbVelocitySolverIterations; i++) {
                if (isConstraintsToSolve) {
                    mConstraintSolver.solveVelocityConstraints(mIslands[islandIndex]);
                }
                if (isContactsToSolve) {
                    mContactSolver.solve();
                }
            }
            if (isContactsToSolve) {
                mContactSolver.storeImpulses();
                mContactSolver.cleanup();
            }
        }
    }

    public void solvePositionCorrection(@Dimensionless DynamicsWorld this) {
        if (mJoints.isEmpty()) {
            return;
        }
        mConstrainedPositions.clear();
        mConstrainedPositions.ensureCapacity(mRigidBodies.size());
        mConstrainedOrientations.clear();
        mConstrainedOrientations.ensureCapacity(mRigidBodies.size());
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mRigidBodies.size(); i++) {
            mConstrainedPositions.add(null);
            mConstrainedOrientations.add(null);
        }
        for (@Dimensionless int islandIndex = ((@Dimensionless int) (0)); islandIndex < mNbIslands; islandIndex++) {
            final @Dimensionless RigidBody @Dimensionless [] bodies = mIslands[islandIndex].getBodies();
            for (@Dimensionless int b = ((@Dimensionless int) (0)); b < mIslands[islandIndex].getNbBodies(); b++) {
                final @Dimensionless int index = mMapBodyToConstrainedVelocityIndex.get(bodies[b]);
                final @Dimensionless Transform transform = bodies[b].getTransform();
                mConstrainedPositions.set(index, new @Dimensionless Vector3(transform.getPosition()));
                mConstrainedOrientations.set(index, new @Dimensionless Quaternion(transform.getOrientation()));
            }
            for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbPositionSolverIterations; i++) {
                mConstraintSolver.solvePositionConstraints(mIslands[islandIndex]);
            }
            for (@Dimensionless int b = ((@Dimensionless int) (0)); b < mIslands[islandIndex].getNbBodies(); b++) {
                final @Dimensionless int index = mMapBodyToConstrainedVelocityIndex.get(bodies[b]);
                final @Dimensionless Vector3 newPosition = mConstrainedPositions.get(index);
                final @Dimensionless Quaternion newOrientation = mConstrainedOrientations.get(index);
                final @Dimensionless Transform newTransform = new @Dimensionless Transform(newPosition, newOrientation.getUnit());
                bodies[b].setTransform(newTransform);
            }
        }
    }

    /**
     * Creates a mobile rigid body and adds it to the physics world. The inertia tensor will be computed from the shape and mass.
     *
     * @param transform The transform (position and orientation) of the body
     * @param mass The mass of the body
     * @param collisionShape The collision shape
     * @return The new rigid body
     */
    public @Dimensionless RigidBody createRigidBody(@Dimensionless DynamicsWorld this, @Dimensionless Transform transform, @Dimensionless float mass, @Dimensionless CollisionShape collisionShape) {
        final @Dimensionless Matrix3x3 inertiaTensor = new @Dimensionless Matrix3x3();
        collisionShape.computeLocalInertiaTensor(inertiaTensor, mass);
        return createRigidBody(transform, mass, inertiaTensor, collisionShape);
    }

    /**
     * Creates a mobile rigid body and adds it to the physics world.
     *
     * @param transform The transform (position and orientation) of the body
     * @param mass The mass of the body
     * @param inertiaTensorLocal The local inertia tensor
     * @param collisionShape The collision shape
     * @return The new rigid body
     */
    public @Dimensionless RigidBody createRigidBody(@Dimensionless DynamicsWorld this, @Dimensionless Transform transform, @Dimensionless float mass, @Dimensionless Matrix3x3 inertiaTensorLocal, @Dimensionless CollisionShape collisionShape) {
        final @Dimensionless CollisionShape newCollisionShape = createCollisionShape(collisionShape);
        final @Dimensionless RigidBody mobileBody = new @Dimensionless RigidBody(transform, mass, inertiaTensorLocal, newCollisionShape, getNextFreeID());
        addRigidBody(mobileBody);
        return mobileBody;
    }

    /**
     * Adds a rigid body to the body collections and the collision detection, even if a tick is in progress.
     *
     * @param body The body to add
     */
    protected void addRigidBodyIgnoreTick(@Dimensionless DynamicsWorld this, RigidBody body) {
        mBodies.add(body);
        mRigidBodies.add(body);
        mCollisionDetection.addBody(body);
    }

    /**
     * Adds a rigid body to the body collections and the collision detection. If a tick is in progress, the body is added at the end.
     *
     * @param body The body to add
     */
    public void addRigidBody(@Dimensionless DynamicsWorld this, @Dimensionless RigidBody body) {
        if (!isTicking) {
            mBodies.add(body);
            mRigidBodies.add(body);
            mCollisionDetection.addBody(body);
        } else {
            mRigidBodiesToAddCache.add(body);
        }
    }

    /**
     * Destroys a rigid body and all the joints to which it belongs.
     *
     * @param rigidBody The rigid body to destroy
     */
    public void destroyRigidBody(@Dimensionless DynamicsWorld this, @Dimensionless RigidBody rigidBody) {
        if (!isTicking) {
            destroyRigidBodyImmediately(rigidBody);
        } else {
            mRigidBodiesToDeleteCache.add(rigidBody);
        }
    }

    protected void destroyRigidBodyImmediately(@Dimensionless DynamicsWorld this, RigidBody rigidBody) {
        mCollisionDetection.removeBody(rigidBody);
        mFreeBodiesIDs.push(rigidBody.getID());
        mBodies.remove(rigidBody);
        mRigidBodies.remove(rigidBody);
        removeCollisionShape(rigidBody.getCollisionShape());
        @Dimensionless
        JointListElement element;
        for (element = rigidBody.getJointsList(); element != null; element = element.getNext()) {
            destroyJoint(element.getJoint());
        }
        rigidBody.resetContactManifoldsList();
    }

    /**
     * Creates a joint between two bodies in the world and returns the new joint.
     *
     * @param jointInfo The information to use for creating the joint
     * @return The new joint
     */
    public @Dimensionless Joint createJoint(@Dimensionless DynamicsWorld this, @Dimensionless JointInfo jointInfo) {
        final @Dimensionless Joint newJoint;
        switch (jointInfo.getType()) {
            case BALLSOCKETJOINT: {
                final @Dimensionless BallAndSocketJointInfo info = (@Dimensionless BallAndSocketJointInfo) jointInfo;
                newJoint = new @Dimensionless BallAndSocketJoint(info);
                break;
            }
            case SLIDERJOINT: {
                final @Dimensionless SliderJointInfo info = (@Dimensionless SliderJointInfo) jointInfo;
                newJoint = new @Dimensionless SliderJoint(info);
                break;
            }
            case HINGEJOINT: {
                final @Dimensionless HingeJointInfo info = (@Dimensionless HingeJointInfo) jointInfo;
                newJoint = new @Dimensionless HingeJoint(info);
                break;
            }
            case FIXEDJOINT: {
                final @Dimensionless FixedJointInfo info = (@Dimensionless FixedJointInfo) jointInfo;
                newJoint = new @Dimensionless FixedJoint(info);
                break;
            }
            default:
                throw new @Dimensionless IllegalArgumentException("Unsupported joint type +" + jointInfo.getType());
        }
        if (!jointInfo.isCollisionEnabled()) {
            mCollisionDetection.addNoCollisionPair(jointInfo.getFirstBody(), jointInfo.getSecondBody());
        }
        mJoints.add(newJoint);
        addJointToBody(newJoint);
        return newJoint;
    }

    /**
     * Destroys a joint.
     *
     * @param joint The joint to destroy
     */
    public void destroyJoint(@Dimensionless DynamicsWorld this, @Dimensionless Joint joint) {
        if (joint == null) {
            throw new @Dimensionless IllegalArgumentException("Joint cannot be null");
        }
        if (!joint.isCollisionEnabled()) {
            mCollisionDetection.removeNoCollisionPair(joint.getFirstBody(), joint.getSecondBody());
        }
        joint.getFirstBody().setIsSleeping(false);
        joint.getSecondBody().setIsSleeping(false);
        mJoints.remove(joint);
        joint.getFirstBody().removeJointFromJointsList(joint);
        joint.getSecondBody().removeJointFromJointsList(joint);
    }

    /**
     * Adds the joint to the list of joints of the two bodies involved in the joint.
     *
     * @param joint The joint to add
     */
    public void addJointToBody(@Dimensionless DynamicsWorld this, @Dimensionless Joint joint) {
        if (joint == null) {
            throw new @Dimensionless IllegalArgumentException("Joint cannot be null");
        }
        final @Dimensionless JointListElement jointListElement1 = new @Dimensionless JointListElement(joint, joint.getFirstBody().getJointsList());
        joint.getFirstBody().setJointsList(jointListElement1);
        final @Dimensionless JointListElement jointListElement2 = new @Dimensionless JointListElement(joint, joint.getSecondBody().getJointsList());
        joint.getSecondBody().setJointsList(jointListElement2);
    }

    /**
     * Adds a contact manifold to the linked list of contact manifolds of the two bodies involved in the corresponding contact.
     *
     * @param contactManifold The contact manifold to add
     * @param body1 The first body in the manifold
     * @param body2 The second body in the manifold
     */
    public void addContactManifoldToBody(@Dimensionless DynamicsWorld this, @Dimensionless ContactManifold contactManifold, @Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        if (contactManifold == null) {
            throw new @Dimensionless IllegalArgumentException("The contact manifold cannot be null");
        }
        final @Dimensionless ContactManifoldListElement listElement1 = new @Dimensionless ContactManifoldListElement(contactManifold, body1.getContactManifoldsLists());
        body1.setContactManifoldsList(listElement1);
        final @Dimensionless ContactManifoldListElement listElement2 = new @Dimensionless ContactManifoldListElement(contactManifold, body2.getContactManifoldsLists());
        body2.setContactManifoldsList(listElement2);
    }

    /**
     * Resets all the contact manifolds linked list of each body.
     */
    public void resetContactManifoldListsOfBodies(@Dimensionless DynamicsWorld this) {
        for (@Dimensionless RigidBody rigidBody : mRigidBodies) {
            rigidBody.resetContactManifoldsList();
        }
    }

    // Computes the islands of awake bodies.
    // An island is an isolated group of rigid bodies that have constraints (joints or contacts)
    // between each other. This method computes the islands at each time step as follows: For each
    // awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
    // (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
    // find all the bodies that are connected with it (the bodies that share joints or contacts with
    // it). Then, we create an island with this group of connected bodies.
    private void computeIslands() {
        final @Dimensionless int nbBodies = mRigidBodies.size();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbIslands; i++) {
            mIslands[i] = null;
        }
        if (mNbIslandsCapacity != nbBodies && nbBodies > ((@Dimensionless int) (0))) {
            mNbIslandsCapacity = nbBodies;
            mIslands = new @Dimensionless Island @Dimensionless [mNbIslandsCapacity];
        }
        mNbIslands = ((@Dimensionless int) (0));
        for (@Dimensionless RigidBody rigidBody : mRigidBodies) {
            rigidBody.setIsAlreadyInIsland(false);
        }
        for (@Dimensionless ContactManifold contactManifold : mContactManifolds) {
            contactManifold.setIsAlreadyInIsland(false);
        }
        for (@Dimensionless Joint joint : mJoints) {
            joint.setIsAlreadyInIsland(false);
        }
        final @Dimensionless RigidBody @Dimensionless [] stackBodiesToVisit = new @Dimensionless RigidBody @Dimensionless [nbBodies];
        for (@Dimensionless RigidBody body : mRigidBodies) {
            if (body.isAlreadyInIsland()) {
                continue;
            }
            if (!body.isMotionEnabled()) {
                continue;
            }
            if (body.isSleeping() || !body.isActive()) {
                continue;
            }
            @Dimensionless
            int stackIndex = ((@Dimensionless int) (0));
            stackBodiesToVisit[stackIndex] = body;
            stackIndex++;
            body.setIsAlreadyInIsland(true);
            mIslands[mNbIslands] = new @Dimensionless Island(nbBodies, mContactManifolds.size(), mJoints.size());
            while (stackIndex > ((@Dimensionless int) (0))) {
                stackIndex--;
                final @Dimensionless RigidBody bodyToVisit = stackBodiesToVisit[stackIndex];
                if (!bodyToVisit.isActive()) {
                    throw new @Dimensionless IllegalStateException("Body to visit isn't active");
                }
                bodyToVisit.setIsSleeping(false);
                mIslands[mNbIslands].addBody(bodyToVisit);
                if (!bodyToVisit.isMotionEnabled()) {
                    continue;
                }
                @Dimensionless
                ContactManifoldListElement contactElement;
                for (contactElement = bodyToVisit.getContactManifoldsLists(); contactElement != null; contactElement = contactElement.getNext()) {
                    final @Dimensionless ContactManifold contactManifold = contactElement.getContactManifold();
                    if (contactManifold.isAlreadyInIsland()) {
                        continue;
                    }
                    mIslands[mNbIslands].addContactManifold(contactManifold);
                    contactManifold.setIsAlreadyInIsland(true);
                    final @Dimensionless RigidBody body1 = (@Dimensionless RigidBody) contactManifold.getFirstBody();
                    final @Dimensionless RigidBody body2 = (@Dimensionless RigidBody) contactManifold.getSecondBody();
                    final @Dimensionless RigidBody otherBody = body1.getID() == bodyToVisit.getID() ? body2 : body1;
                    if (otherBody.isAlreadyInIsland()) {
                        continue;
                    }
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody.setIsAlreadyInIsland(true);
                }
                @Dimensionless
                JointListElement jointElement;
                for (jointElement = bodyToVisit.getJointsList(); jointElement != null; jointElement = jointElement.getNext()) {
                    final @Dimensionless Joint joint = jointElement.getJoint();
                    if (joint.isAlreadyInIsland()) {
                        continue;
                    }
                    mIslands[mNbIslands].addJoint(joint);
                    joint.setIsAlreadyInIsland(true);
                    final @Dimensionless RigidBody body1 = joint.getFirstBody();
                    final @Dimensionless RigidBody body2 = joint.getSecondBody();
                    final @Dimensionless RigidBody otherBody = body1.getID() == bodyToVisit.getID() ? body2 : body1;
                    if (otherBody.isAlreadyInIsland()) {
                        continue;
                    }
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody.setIsAlreadyInIsland(true);
                }
            }
            for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mIslands[mNbIslands].getNbBodies(); i++) {
                if (!mIslands[mNbIslands].getBodies()[i].isMotionEnabled()) {
                    mIslands[mNbIslands].getBodies()[i].setIsAlreadyInIsland(false);
                }
            }
            mNbIslands++;
        }
    }

    // Puts bodies to sleep if needed.
    // For each island, if all the bodies have been almost still for a long enough period of
    // time, we put all the bodies of the island to sleep.
    private void updateSleepingBodies(@Dimensionless DynamicsWorld this) {
        final @Dimensionless float dt = (@Dimensionless float) mTimer.getTimeStep();
        final @Dimensionless float sleepLinearVelocitySquare = mSleepLinearVelocity * mSleepLinearVelocity;
        final @Dimensionless float sleepAngularVelocitySquare = mSleepAngularVelocity * mSleepAngularVelocity;
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbIslands; i++) {
            @Dimensionless
            float minSleepTime = ((@Dimensionless float) (Float.MAX_VALUE));
            final @Dimensionless RigidBody @Dimensionless [] bodies = mIslands[i].getBodies();
            for (@Dimensionless int b = ((@Dimensionless int) (0)); b < mIslands[i].getNbBodies(); b++) {
                if (!bodies[b].isMotionEnabled()) {
                    continue;
                }
                if (bodies[b].getLinearVelocity().lengthSquare() > sleepLinearVelocitySquare || bodies[b].getAngularVelocity().lengthSquare() > sleepAngularVelocitySquare
                        || !bodies[b].isAllowedToSleep()) {
                    bodies[b].setSleepTime(((@Dimensionless int) (0)));
                    minSleepTime = ((@Dimensionless int) (0));
                } else {
                    bodies[b].setSleepTime(bodies[b].getSleepTime() + dt);
                    if (bodies[b].getSleepTime() < minSleepTime) {
                        minSleepTime = bodies[b].getSleepTime();
                    }
                }
            }
            if (minSleepTime >= mTimeBeforeSleep) {
                for (@Dimensionless int b = ((@Dimensionless int) (0)); b < mIslands[i].getNbBodies(); b++) {
                    bodies[b].setIsSleeping(true);
                }
            }
        }
    }

    // Resets the external force and torque applied to the bodies
    private void resetBodiesForceAndTorque(@Dimensionless DynamicsWorld this) {
        for (@Dimensionless RigidBody rigidBody : mRigidBodies) {
            rigidBody.getExternalForce().setToZero();
            rigidBody.getExternalTorque().setToZero();
        }
    }

    @Override
    public void notifyAddedOverlappingPair(@Dimensionless DynamicsWorld this, @Dimensionless BroadPhasePair addedPair) {
        final @Dimensionless IntPair indexPair = addedPair.getBodiesIndexPair();
        final @Dimensionless OverlappingPair newPair = new @Dimensionless OverlappingPair(addedPair.getFirstBody(), addedPair.getSecondBody());
        final @Dimensionless OverlappingPair oldPair = mOverlappingPairs.put(indexPair, newPair);
        if (oldPair != null) {
            throw new @Dimensionless IllegalStateException("overlapping pair was already in the overlapping pairs map");
        }
    }

    @Override
    public void updateOverlappingPair(@Dimensionless DynamicsWorld this, @Dimensionless BroadPhasePair pair) {
        final @Dimensionless IntPair indexPair = pair.getBodiesIndexPair();
        final @Dimensionless OverlappingPair overlappingPair = mOverlappingPairs.get(indexPair);
        overlappingPair.update();
    }

    @Override
    public void notifyRemovedOverlappingPair(@Dimensionless DynamicsWorld this, @Dimensionless BroadPhasePair removedPair) {
        final @Dimensionless IntPair indexPair = removedPair.getBodiesIndexPair();
        mOverlappingPairs.remove(indexPair);
    }

    @Override
    public void notifyNewContact(@Dimensionless DynamicsWorld this, @Dimensionless BroadPhasePair broadPhasePair, @Dimensionless ContactPointInfo contactInfo) {
        final @Dimensionless ContactPoint contact = new @Dimensionless ContactPoint(contactInfo);
        final @Dimensionless IntPair indexPair = broadPhasePair.getBodiesIndexPair();
        final @Dimensionless OverlappingPair overlappingPair = mOverlappingPairs.get(indexPair);
        if (overlappingPair == null) {
            throw new @Dimensionless IllegalArgumentException("broad phase pair is not in the overlapping pairs");
        }
        if (overlappingPair.getNbContactPoints() == ((@Dimensionless int) (0))) {
            if (mEventListener != null) {
                mEventListener.beginContact(contactInfo);
            }
        }
        overlappingPair.addContact(contact);
        mContactManifolds.add(overlappingPair.getContactManifold());
        addContactManifoldToBody(overlappingPair.getContactManifold(), overlappingPair.getFirstBody(), overlappingPair.getSecondBody());
        if (mEventListener != null) {
            mEventListener.newContact(contactInfo);
        }
    }

    /**
     * Enables or disables the sleeping technique.
     *
     * @param isSleepingEnabled The state of the sleeping
     */
    public void enableSleeping(@Dimensionless DynamicsWorld this, @Dimensionless boolean isSleepingEnabled) {
        mIsSleepingEnabled = isSleepingEnabled;
        if (!mIsSleepingEnabled) {
            for (@Dimensionless RigidBody rigidBody : mRigidBodies) {
                rigidBody.setIsSleeping(false);
            }
        }
    }

    /**
     * Returns if the world is currently undertaking a tick
     *
     * @return True if ticking, false if not
     */
    public @Dimensionless boolean isTicking(@Dimensionless DynamicsWorld this) {
        return isTicking;
    }

    /**
     * Disperses the cache of bodies added/removed during the physics tick.
     */
    public void disperseCache(@Dimensionless DynamicsWorld this) {
        mRigidBodiesToAddCache.removeAll(mRigidBodiesToDeleteCache);
        for (@Dimensionless RigidBody body : mRigidBodiesToDeleteCache) {
            destroyRigidBody(body);
        }
        for (@Dimensionless RigidBody body : mRigidBodiesToAddCache) {
            addRigidBody(body);
        }
        mRigidBodiesToAddCache.clear();
        mRigidBodiesToDeleteCache.clear();
    }
}
