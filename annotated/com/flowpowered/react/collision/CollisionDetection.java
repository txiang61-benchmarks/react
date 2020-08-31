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
package com.flowpowered.react.collision;

import units.qual.Dimensionless;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import com.flowpowered.react.Utilities.IntPair;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.collision.broadphase.BroadPhaseAlgorithm;
import com.flowpowered.react.collision.broadphase.PairManager.BodyPair;
import com.flowpowered.react.collision.broadphase.SweepAndPruneAlgorithm;
import com.flowpowered.react.collision.linkedphase.LinkedPhase;
import com.flowpowered.react.collision.narrowphase.GJK.GJKAlgorithm;
import com.flowpowered.react.collision.narrowphase.NarrowPhaseAlgorithm;
import com.flowpowered.react.collision.narrowphase.SphereVsSphereAlgorithm;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.collision.shape.CollisionShape.CollisionShapeType;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.engine.CollisionWorld;
import com.flowpowered.react.engine.linked.LinkedDynamicsWorld;

/**
 * This class computes the collision detection algorithms. We first perform a broad-phase algorithm to know which pairs of bodies can collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between the bodies.
 */
public class CollisionDetection {
    private final @Dimensionless CollisionWorld mWorld;
    private final @Dimensionless Map<@Dimensionless IntPair, @Dimensionless BroadPhasePair> mOverlappingPairs = new @Dimensionless HashMap<>();
    private final @Dimensionless BroadPhaseAlgorithm mBroadPhaseAlgorithm;
    private final @Dimensionless GJKAlgorithm mNarrowPhaseGJKAlgorithm = new @Dimensionless GJKAlgorithm();
    private final @Dimensionless SphereVsSphereAlgorithm mNarrowPhaseSphereVsSphereAlgorithm = new @Dimensionless SphereVsSphereAlgorithm();
    private final @Dimensionless Set<@Dimensionless IntPair> mNoCollisionPairs = new @Dimensionless HashSet<>();
    private final @Dimensionless LinkedPhase mLinkedPhase;

    /**
     * Constructs a new collision detection from the collision world.
     *
     * @param world The world
     */
    public CollisionDetection(@Dimensionless CollisionWorld world) {
        mWorld = world;
        mBroadPhaseAlgorithm = new @Dimensionless SweepAndPruneAlgorithm(this);
        if (world instanceof LinkedDynamicsWorld) {
            mLinkedPhase = new @Dimensionless LinkedPhase((@Dimensionless LinkedDynamicsWorld) mWorld);
        } else {
            mLinkedPhase = null;
        }
    }

    /**
     * Adds a body to the collision detection.
     *
     * @param body The body to add
     */
    public void addBody(@Dimensionless CollisionDetection this, @Dimensionless CollisionBody body) {
        mBroadPhaseAlgorithm.addObject(body, body.getAABB());
    }

    /**
     * Removes a body from the collision detection.
     *
     * @param body The body to remove
     */
    public void removeBody(@Dimensionless CollisionDetection this, @Dimensionless CollisionBody body) {
        mBroadPhaseAlgorithm.removeObject(body);
    }

    /**
     * Add a pair of bodies that cannot collide with each other.
     *
     * @param body1 The first body
     * @param body2 The second body
     */
    public void addNoCollisionPair(@Dimensionless CollisionDetection this, @Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        mNoCollisionPairs.add(BroadPhasePair.computeBodiesIndexPair(body1, body2));
    }

    /**
     * Removes a pair of bodies that cannot collide with each other.
     *
     * @param body1 The first body
     * @param body2 The second body
     */
    public void removeNoCollisionPair(@Dimensionless CollisionDetection this, @Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        mNoCollisionPairs.remove(BroadPhasePair.computeBodiesIndexPair(body1, body2));
    }

    /**
     * Computes the collision detection.
     */
    public void computeCollisionDetection(@Dimensionless CollisionDetection this) {
        if (mLinkedPhase != null) {
            computeLinkedPhase();
        }
        computeBroadPhase();
        computeNarrowPhase();
    }

    // Computes the linked-phase collision detection.
    private void computeLinkedPhase(@Dimensionless CollisionDetection this) {
        final @Dimensionless List<@Dimensionless CollisionBody> bodies = new @Dimensionless ArrayList<>(mWorld.getBodies());
        final @Dimensionless List<@Dimensionless RigidBody> foundBodies = new @Dimensionless ArrayList<>();
        for (@Dimensionless CollisionBody body : bodies) {
            if (!(body instanceof RigidBody)) {
                continue;
            }
            for (@Dimensionless RigidBody rigidBody : mLinkedPhase.getBodiesInRange((@Dimensionless RigidBody) body)) {
                foundBodies.add(rigidBody);
            }
        }
        ((@Dimensionless LinkedDynamicsWorld) mWorld).addLinkedBodies(foundBodies);
    }

    // Computes the broad-phase collision detection.
    private void computeBroadPhase(@Dimensionless CollisionDetection this) {
        for (@Dimensionless CollisionBody body : mWorld.getBodies()) {
            if (body.getHasMoved()) {
                mBroadPhaseAlgorithm.updateObject(body, body.getAABB());
            }
        }
    }

    // Computes the narrow-phase collision detection.
    private void computeNarrowPhase(@Dimensionless CollisionDetection this) {
        for (@Dimensionless Entry<@Dimensionless IntPair, @Dimensionless BroadPhasePair> entry : mOverlappingPairs.entrySet()) {
            final @Dimensionless ContactPointInfo contactInfo = new @Dimensionless ContactPointInfo();
            final @Dimensionless BroadPhasePair pair = entry.getValue();
            if (pair == null) {
                throw new @Dimensionless IllegalStateException("pair cannot be null");
            }
            final @Dimensionless CollisionBody body1 = pair.getFirstBody();
            final @Dimensionless CollisionBody body2 = pair.getSecondBody();
            mWorld.updateOverlappingPair(pair);
            if (mNoCollisionPairs.contains(pair.getBodiesIndexPair())) {
                continue;
            }
            if (body1.isSleeping() && body2.isSleeping()) {
                continue;
            }
            final @Dimensionless NarrowPhaseAlgorithm narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(body1.getCollisionShape(), body2.getCollisionShape());
            narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);
            if (narrowPhaseAlgorithm.testCollision(body1.getCollisionShape(), body1.getTransform(), body2.getCollisionShape(), body2.getTransform(), contactInfo)) {
                contactInfo.setFirstBody((@Dimensionless RigidBody) body1);
                contactInfo.setSecondBody((@Dimensionless RigidBody) body2);
                mWorld.notifyNewContact(pair, contactInfo);
            }
        }
    }

    /**
     * Allows the broad phase to notify the collision detection about an overlapping pair. This method is called by a broad-phase collision detection algorithm.
     *
     * @param addedPair The pair that was added
     */
    public void broadPhaseNotifyAddedOverlappingPair(@Dimensionless CollisionDetection this, BodyPair addedPair) {
        final @Dimensionless IntPair indexPair = addedPair.getBodiesIndexPair();
        final @Dimensionless BroadPhasePair broadPhasePair = new @Dimensionless BroadPhasePair(addedPair.getFirstBody(), addedPair.getSecondBody());
        final @Dimensionless BroadPhasePair old = mOverlappingPairs.put(indexPair, broadPhasePair);
        if (old != null) {
            throw new @Dimensionless IllegalStateException("the pair already existed in the overlapping pairs map");
        }
        mWorld.notifyAddedOverlappingPair(broadPhasePair);
    }

    /**
     * Allows the broad phase to notify the collision detection about a removed overlapping pair.
     *
     * @param removedPair The pair that was removed
     */
    public void broadPhaseNotifyRemovedOverlappingPair(@Dimensionless CollisionDetection this, BodyPair removedPair) {
        final @Dimensionless IntPair indexPair = removedPair.getBodiesIndexPair();
        final @Dimensionless BroadPhasePair broadPhasePair = mOverlappingPairs.get(indexPair);
        if (broadPhasePair == null) {
            throw new @Dimensionless IllegalStateException("the removed pair must be in the map");
        }
        mWorld.notifyRemovedOverlappingPair(broadPhasePair);
        mOverlappingPairs.remove(indexPair);
    }

    // Selects the narrow-phase collision algorithm to use given two collision shapes.
    private @Dimensionless NarrowPhaseAlgorithm selectNarrowPhaseAlgorithm(@Dimensionless CollisionDetection this, @Dimensionless CollisionShape collisionShape1, @Dimensionless CollisionShape collisionShape2) {
        if (collisionShape1.getType() == CollisionShapeType.SPHERE && collisionShape2.getType() == CollisionShapeType.SPHERE) {
            return mNarrowPhaseSphereVsSphereAlgorithm;
        } else {
            return mNarrowPhaseGJKAlgorithm;
        }
    }
}
