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
package com.flowpowered.react.constraint;

import units.qual.Dimensionless;
import com.flowpowered.react.ReactDefaults.JointsPositionCorrectionTechnique;
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.constraint.ConstraintSolver.ConstraintSolverData;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Quaternion;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation between two bodies. This joint has three degrees of freedom. It can be used to create a chain of bodies for instance.
 */
public class BallAndSocketJoint extends Joint {
    private static final @Dimensionless float BETA = ((@Dimensionless float) (0.2f));
    private final @Dimensionless Vector3 mLocalAnchorPointBody1;
    private final @Dimensionless Vector3 mLocalAnchorPointBody2;
    private final @Dimensionless Vector3 mR1World = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR2World = new @Dimensionless Vector3();
    private final @Dimensionless Matrix3x3 mI1 = new @Dimensionless Matrix3x3();
    private final @Dimensionless Matrix3x3 mI2 = new @Dimensionless Matrix3x3();
    private final @Dimensionless Vector3 mBiasVector = new @Dimensionless Vector3();
    private final @Dimensionless Matrix3x3 mInverseMassMatrix = new @Dimensionless Matrix3x3();
    private final @Dimensionless Vector3 mImpulse;

    /**
     * Constructs a new ball and socket joint from provided ball and socket joint info.
     *
     * @param jointInfo The joint info
     */
    public BallAndSocketJoint(BallAndSocketJointInfo jointInfo) {
        super(jointInfo);
        mImpulse = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mLocalAnchorPointBody1 = Transform.multiply(mBody1.getTransform().getInverse(), jointInfo.getAnchorPointWorldSpace());
        mLocalAnchorPointBody2 = Transform.multiply(mBody2.getTransform().getInverse(), jointInfo.getAnchorPointWorldSpace());
    }

    @Override
    public void initBeforeSolve(@Dimensionless BallAndSocketJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        mIndexBody1 = constraintSolverData.getMapBodyToConstrainedVelocityIndex().get(mBody1);
        mIndexBody2 = constraintSolverData.getMapBodyToConstrainedVelocityIndex().get(mBody2);
        final @Dimensionless Vector3 x1 = mBody1.getTransform().getPosition();
        final @Dimensionless Vector3 x2 = mBody2.getTransform().getPosition();
        final @Dimensionless Quaternion orientationBody1 = mBody1.getTransform().getOrientation();
        final @Dimensionless Quaternion orientationBody2 = mBody2.getTransform().getOrientation();
        mI1.set(mBody1.getInertiaTensorInverseWorld());
        mI2.set(mBody2.getInertiaTensorInverseWorld());
        mR1World.set(Quaternion.multiply(orientationBody1, mLocalAnchorPointBody1));
        mR2World.set(Quaternion.multiply(orientationBody2, mLocalAnchorPointBody2));
        final @Dimensionless Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        final @Dimensionless Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);
        @Dimensionless
        float inverseMassBodies = ((@Dimensionless int) (0));
        if (mBody1.isMotionEnabled()) {
            inverseMassBodies += mBody1.getMassInverse();
        }
        if (mBody2.isMotionEnabled()) {
            inverseMassBodies += mBody2.getMassInverse();
        }
        final @Dimensionless Matrix3x3 massMatrix = new @Dimensionless Matrix3x3(
                inverseMassBodies, ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), inverseMassBodies, ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), inverseMassBodies);
        if (mBody1.isMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU1, Matrix3x3.multiply(mI1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU2, Matrix3x3.multiply(mI2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrix.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrix.set(massMatrix.getInverse());
        }
        mBiasVector.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            final @Dimensionless float biasFactor = BETA / constraintSolverData.getTimeStep();
            mBiasVector.set(Vector3.multiply(biasFactor, Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World)));
        }
        if (!constraintSolverData.isWarmStartingActive()) {
            mImpulse.setToZero();
        }
    }

    @Override
    public void warmstart(@Dimensionless BallAndSocketJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        final @Dimensionless Vector3 v1 = constraintSolverData.getLinearVelocities()[mIndexBody1];
        final @Dimensionless Vector3 v2 = constraintSolverData.getLinearVelocities()[mIndexBody2];
        final @Dimensionless Vector3 w1 = constraintSolverData.getAngularVelocities()[mIndexBody1];
        final @Dimensionless Vector3 w2 = constraintSolverData.getAngularVelocities()[mIndexBody2];
        final @Dimensionless float inverseMassBody1 = mBody1.getMassInverse();
        final @Dimensionless float inverseMassBody2 = mBody2.getMassInverse();
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.negate(mImpulse);
            final @Dimensionless Vector3 angularImpulseBody1 = mImpulse.cross(mR1World);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = mImpulse;
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.negate(mImpulse.cross(mR2World));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
    }

    @Override
    public void solveVelocityConstraint(@Dimensionless BallAndSocketJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        final @Dimensionless Vector3 v1 = constraintSolverData.getLinearVelocities()[mIndexBody1];
        final @Dimensionless Vector3 v2 = constraintSolverData.getLinearVelocities()[mIndexBody2];
        final @Dimensionless Vector3 w1 = constraintSolverData.getAngularVelocities()[mIndexBody1];
        final @Dimensionless Vector3 w2 = constraintSolverData.getAngularVelocities()[mIndexBody2];
        @Dimensionless
        float inverseMassBody1 = mBody1.getMassInverse();
        @Dimensionless
        float inverseMassBody2 = mBody2.getMassInverse();
        final @Dimensionless Vector3 Jv = Vector3.subtract(Vector3.subtract(Vector3.add(v2, w2.cross(mR2World)), v1), w1.cross(mR1World));
        final @Dimensionless Vector3 deltaLambda = Matrix3x3.multiply(mInverseMassMatrix, Vector3.subtract(Vector3.negate(Jv), mBiasVector));
        mImpulse.add(deltaLambda);
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.negate(deltaLambda);
            final @Dimensionless Vector3 angularImpulseBody1 = deltaLambda.cross(mR1World);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = deltaLambda;
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.negate(deltaLambda.cross(mR2World));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
    }

    @Override
    public void solvePositionConstraint(@Dimensionless BallAndSocketJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        if (mPositionCorrectionTechnique != JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL) {
            return;
        }
        final @Dimensionless Vector3 x1 = constraintSolverData.getPositions().get(mIndexBody1);
        final @Dimensionless Vector3 x2 = constraintSolverData.getPositions().get(mIndexBody2);
        final @Dimensionless Quaternion q1 = constraintSolverData.getOrientations().get(mIndexBody1);
        final @Dimensionless Quaternion q2 = constraintSolverData.getOrientations().get(mIndexBody2);
        final @Dimensionless float inverseMassBody1 = mBody1.getMassInverse();
        final @Dimensionless float inverseMassBody2 = mBody2.getMassInverse();
        mI1.set(mBody1.getInertiaTensorInverseWorld());
        mI2.set(mBody2.getInertiaTensorInverseWorld());
        mR1World.set(Quaternion.multiply(q1, mLocalAnchorPointBody1));
        mR2World.set(Quaternion.multiply(q2, mLocalAnchorPointBody2));
        final @Dimensionless Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        final @Dimensionless Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);
        @Dimensionless
        float inverseMassBodies = ((@Dimensionless int) (0));
        if (mBody1.isMotionEnabled()) {
            inverseMassBodies += inverseMassBody1;
        }
        if (mBody2.isMotionEnabled()) {
            inverseMassBodies += inverseMassBody2;
        }
        final @Dimensionless Matrix3x3 massMatrix = new @Dimensionless Matrix3x3(
                inverseMassBodies, ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), inverseMassBodies, ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), inverseMassBodies);
        if (mBody1.isMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU1, Matrix3x3.multiply(mI1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU2, Matrix3x3.multiply(mI2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrix.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrix.set(massMatrix.getInverse());
        }
        final @Dimensionless Vector3 constraintError = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World);
        final @Dimensionless Vector3 lambda = Matrix3x3.multiply(mInverseMassMatrix, Vector3.negate(constraintError));
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.negate(lambda);
            final @Dimensionless Vector3 angularImpulseBody1 = lambda.cross(mR1World);
            final @Dimensionless Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
            final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            x1.add(v1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = lambda;
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.negate(lambda.cross(mR2World));
            final @Dimensionless Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
            final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            x2.add(v2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w2), q2), ((@Dimensionless float) (0.5f))));
            q2.normalize();
        }
    }

    /**
     * This structure is used to gather the information needed to create a ball-and-socket joint. This structure will be used to create the actual ball-and-socket joint.
     */
    public static class BallAndSocketJointInfo extends JointInfo {
        private final @Dimensionless Vector3 anchorPointWorldSpace = new @Dimensionless Vector3();

        /**
         * Constructs a new ball and socket joint info from both bodies and the initial anchor point in world space.
         *
         * @param rigidBody1 The first body
         * @param rigidBody2 The second body
         * @param initAnchorPointWorldSpace The anchor point in world space
         */
        public BallAndSocketJointInfo(BallAndSocketJoint.@Dimensionless BallAndSocketJointInfo this, @Dimensionless RigidBody rigidBody1, @Dimensionless RigidBody rigidBody2, @Dimensionless Vector3 initAnchorPointWorldSpace) {
            super(rigidBody1, rigidBody2, JointType.BALLSOCKETJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
        }

        /**
         * Returns the anchor point in world space.
         *
         * @return The anchor point in world space
         */
        public @Dimensionless Vector3 getAnchorPointWorldSpace(BallAndSocketJoint.@Dimensionless BallAndSocketJointInfo this) {
            return anchorPointWorldSpace;
        }
    }
}
