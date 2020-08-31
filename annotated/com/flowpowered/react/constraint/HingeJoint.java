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
import units.qual.rad;
import units.qual.UnitsRep;
import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.ReactDefaults.JointsPositionCorrectionTechnique;
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.constraint.ConstraintSolver.ConstraintSolverData;
import com.flowpowered.react.math.Mathematics;
import com.flowpowered.react.math.Matrix2x2;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Quaternion;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector2;
import com.flowpowered.react.math.Vector3;

/**
 * This class represents a hinge joint that allows arbitrary rotation between two bodies around a single axis. This joint has one degree of freedom. It can be useful to simulate doors or pendulums.
 */
public class HingeJoint extends Joint {
    private static final @Dimensionless float BETA = ((@Dimensionless float) (0.2f));
    private final @Dimensionless Vector3 mLocalAnchorPointBody1;
    private final @Dimensionless Vector3 mLocalAnchorPointBody2;
    private final @Dimensionless Vector3 mHingeLocalAxisBody1;
    private final @Dimensionless Vector3 mHingeLocalAxisBody2;
    private final @Dimensionless Matrix3x3 mI1 = new @Dimensionless Matrix3x3();
    private final @Dimensionless Matrix3x3 mI2 = new @Dimensionless Matrix3x3();
    private final @Dimensionless Vector3 mA1 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR1World = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR2World = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mB2CrossA1 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mC2CrossA1 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mImpulseTranslation;
    private final @Dimensionless Vector2 mImpulseRotation;
    private @Dimensionless float mImpulseLowerLimit;
    private @Dimensionless float mImpulseUpperLimit;
    private @Dimensionless float mImpulseMotor;
    private final @Dimensionless Matrix3x3 mInverseMassMatrixTranslation = new @Dimensionless Matrix3x3();
    private final @Dimensionless Matrix2x2 mInverseMassMatrixRotation = new @Dimensionless Matrix2x2();
    private @Dimensionless float mInverseMassMatrixLimitMotor;
    private @Dimensionless float mInverseMassMatrixMotor;
    private final @Dimensionless Vector3 mBTranslation = new @Dimensionless Vector3();
    private final @Dimensionless Vector2 mBRotation = new @Dimensionless Vector2();
    private @Dimensionless float mBLowerLimit;
    private @Dimensionless float mBUpperLimit;
    private final @Dimensionless Quaternion mInitOrientationDifferenceInv;
    private @Dimensionless boolean mIsLimitEnabled;
    private @Dimensionless boolean mIsMotorEnabled;
    private @Dimensionless float mLowerLimit;
    private @Dimensionless float mUpperLimit;
    private @Dimensionless boolean mIsLowerLimitViolated;
    private @Dimensionless boolean mIsUpperLimitViolated;
    private @Dimensionless float mMotorSpeed;
    private @Dimensionless float mMaxMotorTorque;

    public HingeJoint(HingeJointInfo jointInfo) {
        super(jointInfo);
        if (jointInfo.getMinAngleLimit() > ((@Dimensionless int) (0)) || jointInfo.getMinAngleLimit() < ((@Dimensionless int) (-2)) * ((@Dimensionless double) (Math.PI))) {
            throw new @Dimensionless IllegalArgumentException("Lower limit must be smaller or equal to 0 and greater or equal to -2 * PI");
        }
        if (jointInfo.getMaxAngleLimit() < ((@Dimensionless int) (0)) || jointInfo.getMaxAngleLimit() > ((@Dimensionless int) (2)) * ((@Dimensionless double) (Math.PI))) {
            throw new @Dimensionless IllegalArgumentException("Upper limit must be greater or equal to 0 and smaller or equal to 2 * PI");
        }
        mImpulseTranslation = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mImpulseRotation = new @Dimensionless Vector2(((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mImpulseLowerLimit = ((@Dimensionless int) (0));
        mImpulseUpperLimit = ((@Dimensionless int) (0));
        mImpulseMotor = ((@Dimensionless int) (0));
        mIsLimitEnabled = jointInfo.isLimitEnabled();
        mIsMotorEnabled = jointInfo.isMotorEnabled();
        mLowerLimit = jointInfo.getMinAngleLimit();
        mUpperLimit = jointInfo.getMaxAngleLimit();
        mIsLowerLimitViolated = false;
        mIsUpperLimitViolated = false;
        mMotorSpeed = jointInfo.getMotorSpeed();
        mMaxMotorTorque = jointInfo.getMaxMotorTorque();
        final @Dimensionless Transform transform1 = mBody1.getTransform();
        final @Dimensionless Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = Transform.multiply(transform1.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mLocalAnchorPointBody2 = Transform.multiply(transform2.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mHingeLocalAxisBody1 = Quaternion.multiply(transform1.getOrientation().getInverse(), jointInfo.getRotationAxisWorld());
        mHingeLocalAxisBody2 = Quaternion.multiply(transform2.getOrientation().getInverse(), jointInfo.getRotationAxisWorld());
        mHingeLocalAxisBody1.normalize();
        mHingeLocalAxisBody2.normalize();
        mInitOrientationDifferenceInv = Quaternion.multiply(transform2.getOrientation(), transform1.getOrientation().getInverse());
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();
    }

    @Override
    public void initBeforeSolve(@Dimensionless HingeJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
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
        final @Dimensionless float hingeAngle = computeCurrentHingeAngle(orientationBody1, orientationBody2);
        final @Dimensionless float lowerLimitError = hingeAngle - mLowerLimit;
        final @Dimensionless float upperLimitError = mUpperLimit - hingeAngle;
        final @Dimensionless boolean oldIsLowerLimitViolated = mIsLowerLimitViolated;
        mIsLowerLimitViolated = lowerLimitError <= ((@Dimensionless int) (0));
        if (mIsLowerLimitViolated != oldIsLowerLimitViolated) {
            mImpulseLowerLimit = ((@Dimensionless int) (0));
        }
        final @Dimensionless boolean oldIsUpperLimitViolated = mIsUpperLimitViolated;
        mIsUpperLimitViolated = upperLimitError <= ((@Dimensionless int) (0));
        if (mIsUpperLimitViolated != oldIsUpperLimitViolated) {
            mImpulseUpperLimit = ((@Dimensionless int) (0));
        }
        mA1.set(Quaternion.multiply(orientationBody1, mHingeLocalAxisBody1));
        final @Dimensionless Vector3 a2 = Quaternion.multiply(orientationBody2, mHingeLocalAxisBody2);
        mA1.normalize();
        a2.normalize();
        final @Dimensionless Vector3 b2 = a2.getOneUnitOrthogonalVector();
        final @Dimensionless Vector3 c2 = a2.cross(b2);
        mB2CrossA1.set(b2.cross(mA1));
        mC2CrossA1.set(c2.cross(mA1));
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
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation.set(massMatrix.getInverse());
        }
        mBTranslation.setToZero();
        final @Dimensionless float biasFactor = BETA / constraintSolverData.getTimeStep();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation.set(Vector3.multiply(biasFactor, Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World)));
        }
        final @Dimensionless Vector3 I1B2CrossA1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I1C2CrossA1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I2B2CrossA1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I2C2CrossA1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        if (mBody1.isMotionEnabled()) {
            I1B2CrossA1.set(Matrix3x3.multiply(mI1, mB2CrossA1));
            I1C2CrossA1.set(Matrix3x3.multiply(mI1, mC2CrossA1));
        }
        if (mBody2.isMotionEnabled()) {
            I2B2CrossA1.set(Matrix3x3.multiply(mI2, mB2CrossA1));
            I2C2CrossA1.set(Matrix3x3.multiply(mI2, mC2CrossA1));
        }
        final @Dimensionless float el11 = mB2CrossA1.dot(I1B2CrossA1) + mB2CrossA1.dot(I2B2CrossA1);
        final @Dimensionless float el12 = mB2CrossA1.dot(I1C2CrossA1) + mB2CrossA1.dot(I2C2CrossA1);
        final @Dimensionless float el21 = mC2CrossA1.dot(I1B2CrossA1) + mC2CrossA1.dot(I2B2CrossA1);
        final @Dimensionless float el22 = mC2CrossA1.dot(I1C2CrossA1) + mC2CrossA1.dot(I2C2CrossA1);
        final @Dimensionless Matrix2x2 matrixKRotation = new @Dimensionless Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation.set(matrixKRotation.getInverse());
        }
        mBRotation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBRotation.set(Vector2.multiply(biasFactor, new @Dimensionless Vector2(mA1.dot(b2), mA1.dot(c2))));
        }
        if (!constraintSolverData.isWarmStartingActive()) {
            mImpulseTranslation.setToZero();
            mImpulseRotation.setToZero();
            mImpulseLowerLimit = ((@Dimensionless int) (0));
            mImpulseUpperLimit = ((@Dimensionless int) (0));
            mImpulseMotor = ((@Dimensionless int) (0));
        }
        if (mIsMotorEnabled || (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated))) {
            mInverseMassMatrixLimitMotor = ((@Dimensionless int) (0));
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.multiply(mI1, mA1));
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.multiply(mI2, mA1));
            }
            mInverseMassMatrixLimitMotor = mInverseMassMatrixLimitMotor > ((@Dimensionless int) (0)) ? ((@Dimensionless int) (1)) / mInverseMassMatrixLimitMotor : ((@Dimensionless int) (0));
            if (mIsLimitEnabled) {
                mBLowerLimit = ((@Dimensionless int) (0));
                if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                    mBLowerLimit = biasFactor * lowerLimitError;
                }
                mBUpperLimit = ((@Dimensionless int) (0));
                if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                    mBUpperLimit = biasFactor * upperLimitError;
                }
            }
        }
    }

    @Override
    public void warmstart(@Dimensionless HingeJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        final @Dimensionless Vector3 v1 = constraintSolverData.getLinearVelocities()[mIndexBody1];
        final @Dimensionless Vector3 v2 = constraintSolverData.getLinearVelocities()[mIndexBody2];
        final @Dimensionless Vector3 w1 = constraintSolverData.getAngularVelocities()[mIndexBody1];
        final @Dimensionless Vector3 w2 = constraintSolverData.getAngularVelocities()[mIndexBody2];
        final @Dimensionless float inverseMassBody1 = mBody1.getMassInverse();
        final @Dimensionless float inverseMassBody2 = mBody2.getMassInverse();
        final @Dimensionless Vector3 rotationImpulse = Vector3.subtract(Vector3.multiply(Vector3.negate(mB2CrossA1), mImpulseRotation.getX()), Vector3.multiply(mC2CrossA1, mImpulseRotation.getY()));
        final @Dimensionless Vector3 limitsImpulse = Vector3.multiply(mImpulseUpperLimit - mImpulseLowerLimit, mA1);
        final @Dimensionless Vector3 motorImpulse = Vector3.multiply(-mImpulseMotor, mA1);
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.negate(mImpulseTranslation);
            final @Dimensionless Vector3 angularImpulseBody1 = mImpulseTranslation.cross(mR1World);
            angularImpulseBody1.add(rotationImpulse);
            angularImpulseBody1.add(limitsImpulse);
            angularImpulseBody1.add(motorImpulse);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = mImpulseTranslation;
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.negate(mImpulseTranslation.cross(mR2World));
            angularImpulseBody2.add(Vector3.negate(rotationImpulse));
            angularImpulseBody2.add(Vector3.negate(limitsImpulse));
            angularImpulseBody2.add(Vector3.negate(motorImpulse));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
    }

    @Override
    public void solveVelocityConstraint(@Dimensionless HingeJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        final @Dimensionless Vector3 v1 = constraintSolverData.getLinearVelocities()[mIndexBody1];
        final @Dimensionless Vector3 v2 = constraintSolverData.getLinearVelocities()[mIndexBody2];
        final @Dimensionless Vector3 w1 = constraintSolverData.getAngularVelocities()[mIndexBody1];
        final @Dimensionless Vector3 w2 = constraintSolverData.getAngularVelocities()[mIndexBody2];
        final @Dimensionless float inverseMassBody1 = mBody1.getMassInverse();
        final @Dimensionless float inverseMassBody2 = mBody2.getMassInverse();
        final @Dimensionless Vector3 JvTranslation = Vector3.subtract(Vector3.subtract(Vector3.add(v2, w2.cross(mR2World)), v1), w1.cross(mR1World));
        final @Dimensionless Vector3 deltaLambdaTranslation = Matrix3x3.multiply(mInverseMassMatrixTranslation, Vector3.subtract(Vector3.negate(JvTranslation), mBTranslation));
        mImpulseTranslation.add(deltaLambdaTranslation);
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.negate(deltaLambdaTranslation);
            final @Dimensionless Vector3 angularImpulseBody1 = deltaLambdaTranslation.cross(mR1World);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = deltaLambdaTranslation;
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.negate(deltaLambdaTranslation.cross(mR2World));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
        final @Dimensionless Vector2 JvRotation = new @Dimensionless Vector2(-mB2CrossA1.dot(w1) + mB2CrossA1.dot(w2), -mC2CrossA1.dot(w1) + mC2CrossA1.dot(w2));
        final @Dimensionless Vector2 deltaLambdaRotation = Matrix2x2.multiply(mInverseMassMatrixRotation, Vector2.subtract(Vector2.negate(JvRotation), mBRotation));
        mImpulseRotation.add(deltaLambdaRotation);
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mB2CrossA1), deltaLambdaRotation.getX()), Vector3.multiply(mC2CrossA1, deltaLambdaRotation.getY()));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mB2CrossA1, deltaLambdaRotation.getX()), Vector3.multiply(mC2CrossA1, deltaLambdaRotation.getY()));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
        if (mIsLimitEnabled) {
            if (mIsLowerLimitViolated) {
                final @Dimensionless float JvLowerLimit = Vector3.subtract(w2, w1).dot(mA1);
                @Dimensionless
                float deltaLambdaLower = mInverseMassMatrixLimitMotor * (-JvLowerLimit - mBLowerLimit);
                final @Dimensionless float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = Math.max(mImpulseLowerLimit + deltaLambdaLower, ((@Dimensionless int) (0)));
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;
                if (mBody1.isMotionEnabled()) {
                    final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(-deltaLambdaLower, mA1);
                    w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
                }
                if (mBody2.isMotionEnabled()) {
                    final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(deltaLambdaLower, mA1);
                    w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
                }
            }
            if (mIsUpperLimitViolated) {
                final @Dimensionless float JvUpperLimit = -Vector3.subtract(w2, w1).dot(mA1);
                @Dimensionless
                float deltaLambdaUpper = mInverseMassMatrixLimitMotor * (-JvUpperLimit - mBUpperLimit);
                final @Dimensionless float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = Math.max(mImpulseUpperLimit + deltaLambdaUpper, ((@Dimensionless int) (0)));
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;
                if (mBody1.isMotionEnabled()) {
                    final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(deltaLambdaUpper, mA1);
                    w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
                }
                if (mBody2.isMotionEnabled()) {
                    final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(-deltaLambdaUpper, mA1);
                    w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
                }
            }
        }
        if (mIsMotorEnabled) {
            final @Dimensionless float JvMotor = mA1.dot(Vector3.subtract(w1, w2));
            final @Dimensionless float maxMotorImpulse = mMaxMotorTorque * constraintSolverData.getTimeStep();
            @Dimensionless
            float deltaLambdaMotor = mInverseMassMatrixLimitMotor * (-JvMotor - mMotorSpeed);
            final @Dimensionless float lambdaTemp = mImpulseMotor;
            mImpulseMotor = Mathematics.clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;
            if (mBody1.isMotionEnabled()) {
                final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(-deltaLambdaMotor, mA1);
                w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
            }
            if (mBody2.isMotionEnabled()) {
                final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(deltaLambdaMotor, mA1);
                w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
            }
        }
    }

    @Override
    public void solvePositionConstraint(@Dimensionless HingeJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
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
        final @Dimensionless float hingeAngle = computeCurrentHingeAngle(q1, q2);
        final @Dimensionless float lowerLimitError = hingeAngle - mLowerLimit;
        final @Dimensionless float upperLimitError = mUpperLimit - hingeAngle;
        mIsLowerLimitViolated = lowerLimitError <= ((@Dimensionless int) (0));
        mIsUpperLimitViolated = upperLimitError <= ((@Dimensionless int) (0));
        mA1.set(Quaternion.multiply(q1, mHingeLocalAxisBody1));
        final @Dimensionless Vector3 a2 = Quaternion.multiply(q2, mHingeLocalAxisBody2);
        mA1.normalize();
        a2.normalize();
        final @Dimensionless Vector3 b2 = a2.getOneUnitOrthogonalVector();
        final @Dimensionless Vector3 c2 = a2.cross(b2);
        mB2CrossA1.set(b2.cross(mA1));
        mC2CrossA1.set(c2.cross(mA1));
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
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation.set(massMatrix.getInverse());
        }
        final @Dimensionless Vector3 errorTranslation = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World);
        final @Dimensionless Vector3 lambdaTranslation = Matrix3x3.multiply(mInverseMassMatrixTranslation, Vector3.negate(errorTranslation));
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.negate(lambdaTranslation);
            final @Dimensionless Vector3 angularImpulseBody1 = lambdaTranslation.cross(mR1World);
            final @Dimensionless Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
            final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            x1.add(v1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = lambdaTranslation;
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.negate(lambdaTranslation.cross(mR2World));
            final @Dimensionless Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
            final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            x2.add(v2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w2), q2), ((@Dimensionless float) (0.5f))));
            q2.normalize();
        }
        final @Dimensionless Vector3 I1B2CrossA1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I1C2CrossA1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I2B2CrossA1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I2C2CrossA1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        if (mBody1.isMotionEnabled()) {
            I1B2CrossA1.set(Matrix3x3.multiply(mI1, mB2CrossA1));
            I1C2CrossA1.set(Matrix3x3.multiply(mI1, mC2CrossA1));
        }
        if (mBody2.isMotionEnabled()) {
            I2B2CrossA1.set(Matrix3x3.multiply(mI2, mB2CrossA1));
            I2C2CrossA1.set(Matrix3x3.multiply(mI2, mC2CrossA1));
        }
        final @Dimensionless float el11 = mB2CrossA1.dot(I1B2CrossA1) + mB2CrossA1.dot(I2B2CrossA1);
        final @Dimensionless float el12 = mB2CrossA1.dot(I1C2CrossA1) + mB2CrossA1.dot(I2C2CrossA1);
        final @Dimensionless float el21 = mC2CrossA1.dot(I1B2CrossA1) + mC2CrossA1.dot(I2B2CrossA1);
        final @Dimensionless float el22 = mC2CrossA1.dot(I1C2CrossA1) + mC2CrossA1.dot(I2C2CrossA1);
        final @Dimensionless Matrix2x2 matrixKRotation = new @Dimensionless Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation.set(matrixKRotation.getInverse());
        }
        final @Dimensionless Vector2 errorRotation = new @Dimensionless Vector2(mA1.dot(b2), mA1.dot(c2));
        final @Dimensionless Vector2 lambdaRotation = Matrix2x2.multiply(mInverseMassMatrixRotation, Vector2.negate(errorRotation));
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mB2CrossA1), lambdaRotation.getX()), Vector3.multiply(mC2CrossA1, lambdaRotation.getY()));
            final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mB2CrossA1, lambdaRotation.getX()), Vector3.multiply(mC2CrossA1, lambdaRotation.getY()));
            final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w2), q2), ((@Dimensionless float) (0.5f))));
            q2.normalize();
        }
        if (mIsLimitEnabled) {
            if (mIsLowerLimitViolated || mIsUpperLimitViolated) {
                mInverseMassMatrixLimitMotor = ((@Dimensionless int) (0));
                if (mBody1.isMotionEnabled()) {
                    mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.multiply(mI1, mA1));
                }
                if (mBody2.isMotionEnabled()) {
                    mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.multiply(mI2, mA1));
                }
                mInverseMassMatrixLimitMotor = mInverseMassMatrixLimitMotor > ((@Dimensionless int) (0)) ? ((@Dimensionless int) (1)) / mInverseMassMatrixLimitMotor : ((@Dimensionless int) (0));
            }
            if (mIsLowerLimitViolated) {
                final @Dimensionless float lambdaLowerLimit = mInverseMassMatrixLimitMotor * -lowerLimitError;
                if (mBody1.isMotionEnabled()) {
                    final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(-lambdaLowerLimit, mA1);
                    final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
                    q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {
                    final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(lambdaLowerLimit, mA1);
                    final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
                    q2.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w2), q2), ((@Dimensionless float) (0.5f))));
                    q2.normalize();
                }
            }
            if (mIsUpperLimitViolated) {
                final @Dimensionless float lambdaUpperLimit = mInverseMassMatrixLimitMotor * -upperLimitError;
                if (mBody1.isMotionEnabled()) {
                    final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(lambdaUpperLimit, mA1);
                    final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
                    q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {
                    final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(-lambdaUpperLimit, mA1);
                    final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
                    q2.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w2), q2), ((@Dimensionless float) (0.5f))));
                    q2.normalize();
                }
            }
        }
    }

    /**
     * Enables or disables the joint limits.
     *
     * @param isLimitEnabled The new state of the joint limits
     */
    public void enableLimit(@Dimensionless HingeJoint this, @Dimensionless boolean isLimitEnabled) {
        if (isLimitEnabled != mIsLimitEnabled) {
            mIsLimitEnabled = isLimitEnabled;
            resetLimits();
        }
    }

    /**
     * Enables or disables the joint motor.
     *
     * @param isMotorEnabled The new state of the joint motor
     */
    public void enableMotor(@Dimensionless HingeJoint this, @Dimensionless boolean isMotorEnabled) {
        mIsMotorEnabled = isMotorEnabled;
        mImpulseMotor = ((@Dimensionless int) (0));
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    /**
     * Sets the minimum angle limit.
     *
     * @param lowerLimit The minimum limit
     */
    public void setMinAngleLimit(@Dimensionless HingeJoint this, @Dimensionless float lowerLimit) {
        if (lowerLimit > ((@Dimensionless int) (0)) || lowerLimit < ((@Dimensionless int) (-2)) * ((@Dimensionless double) (Math.PI))) {
            throw new @Dimensionless IllegalArgumentException("Lower limit must be smaller or equal to 0 and greater or equal to -2 * PI");
        }
        if (lowerLimit != mLowerLimit) {
            mLowerLimit = lowerLimit;
            resetLimits();
        }
    }

    /**
     * Sets the maximum angle limit.
     *
     * @param upperLimit The maximum limit
     */
    public void setMaxAngleLimit(@Dimensionless HingeJoint this, @Dimensionless float upperLimit) {
        if (upperLimit < ((@Dimensionless int) (0)) || upperLimit > ((@Dimensionless int) (2)) * ((@Dimensionless double) (Math.PI))) {
            throw new @Dimensionless IllegalArgumentException("Upper limit must be greater or equal to 0 and smaller or equal to 2 * PI");
        }
        if (upperLimit != mUpperLimit) {
            mUpperLimit = upperLimit;
            resetLimits();
        }
    }

    // Resets the limits.
    private void resetLimits(@Dimensionless HingeJoint this) {
        mImpulseLowerLimit = ((@Dimensionless int) (0));
        mImpulseUpperLimit = ((@Dimensionless int) (0));
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    /**
     * Sets the motor speed.
     *
     * @param motorSpeed The motor speed
     */
    public void setMotorSpeed(@Dimensionless HingeJoint this, @Dimensionless float motorSpeed) {
        if (motorSpeed != mMotorSpeed) {
            mMotorSpeed = motorSpeed;
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

    /**
     * Sets the maximum motor torque.
     *
     * @param maxMotorForce The maximum motor torque
     */
    public void setMaxMotorForce(@Dimensionless HingeJoint this, @Dimensionless float maxMotorForce) {
        if (mMaxMotorTorque < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Max motor torque must be greater or equal to 0");
        }
        if (maxMotorForce != mMaxMotorTorque) {
            mMaxMotorTorque = maxMotorForce;
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

    /**
     * Return true if the limits or the joint are enabled.
     *
     * @return Whether or not the limits are enabled
     */
    public @Dimensionless boolean isLimitEnabled(@Dimensionless HingeJoint this) {
        return mIsLimitEnabled;
    }

    /**
     * Returns true if the motor of the joint is enabled.
     *
     * @return Whether or not the motor is enabled
     */
    public @Dimensionless boolean isMotorEnabled(@Dimensionless HingeJoint this) {
        return mIsMotorEnabled;
    }

    /**
     * Returns the minimum limit.
     *
     * @return The minimum limit
     */
    public @Dimensionless float getMinAngleLimit(@Dimensionless HingeJoint this) {
        return mLowerLimit;
    }

    /**
     * Returns the maximum limit.
     *
     * @return The maximum limit
     */
    public @Dimensionless float getMaxAngleLimit(@Dimensionless HingeJoint this) {
        return mUpperLimit;
    }

    /**
     * Returns the motor speed.
     *
     * @return The motor speed
     */
    public @Dimensionless float getMotorSpeed(@Dimensionless HingeJoint this) {
        return mMotorSpeed;
    }

    /**
     * Returns the maximum motor torque.
     *
     * @return The maximum motor torque
     */
    public @Dimensionless float getMaxMotorTorque(@Dimensionless HingeJoint this) {
        return mMaxMotorTorque;
    }

    /**
     * Returns the intensity of the current torque applied for the joint motor.
     *
     * @param timeStep The simulation time step
     * @return The motor torque for the time step
     */
    public @Dimensionless float getMotorTorque(@Dimensionless HingeJoint this, @Dimensionless float timeStep) {
        return mImpulseMotor / timeStep;
    }

    // Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi].
    private @Dimensionless float computeNormalizedAngle(@Dimensionless HingeJoint this, @Dimensionless float angle) {
        angle = angle % ReactDefaults.PI_TIMES_2;
        if (angle < - ((@Dimensionless double) (Math.PI))) {
            return angle + ReactDefaults.PI_TIMES_2;
        } else if (angle > ((@Dimensionless double) (Math.PI))) {
            return angle - ReactDefaults.PI_TIMES_2;
        } else {
            return angle;
        }
    }

    // Given an "inputAngle" in the range [-pi, pi], this method returns an
    // angle (modulo 2 * pi) in the range [-2 * pi; 2 * pi] that is closest to one of the two angle limits in arguments.
    private @Dimensionless float computeCorrespondingAngleNearLimits(@Dimensionless HingeJoint this, @Dimensionless float inputAngle, @Dimensionless float lowerLimitAngle, @Dimensionless float upperLimitAngle) {
        if (upperLimitAngle <= lowerLimitAngle) {
            return inputAngle;
        } else if (inputAngle > upperLimitAngle) {
            final @Dimensionless float diffToUpperLimit = Math.abs(computeNormalizedAngle(inputAngle - upperLimitAngle));
            final @Dimensionless float diffToLowerLimit = Math.abs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
            return diffToUpperLimit > diffToLowerLimit ? inputAngle - ReactDefaults.PI_TIMES_2 : inputAngle;
        } else if (inputAngle < lowerLimitAngle) {
            final @Dimensionless float diffToUpperLimit = Math.abs(computeNormalizedAngle(upperLimitAngle - inputAngle));
            final @Dimensionless float diffToLowerLimit = Math.abs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
            return diffToUpperLimit > diffToLowerLimit ? inputAngle : inputAngle + ReactDefaults.PI_TIMES_2;
        } else {
            return inputAngle;
        }
    }

    // Computes the current angle around the hinge axis.
    private @Dimensionless float computeCurrentHingeAngle(@Dimensionless HingeJoint this, @Dimensionless Quaternion orientationBody1, @Dimensionless Quaternion orientationBody2) {
        @Dimensionless
        float hingeAngle;
        final @Dimensionless Quaternion currentOrientationDiff = Quaternion.multiply(orientationBody2, orientationBody1.getInverse());
        currentOrientationDiff.normalize();
        final @Dimensionless Quaternion relativeRotation = Quaternion.multiply(currentOrientationDiff, mInitOrientationDifferenceInv);
        relativeRotation.normalize();
        final @Dimensionless float cosHalfAngle = relativeRotation.getW();
        final @Dimensionless float sinHalfAngleAbs = relativeRotation.getVectorV().length();
        final @Dimensionless float dotProduct = relativeRotation.getVectorV().dot(mA1);
        if (dotProduct >= ((@Dimensionless int) (0))) {
            hingeAngle = ((@UnitsRep(top=false, bot=false, prefixExponent=0, baseUnitComponents={@units.qual.BUC(unit="C", exponent=0), @units.qual.BUC(unit="K", exponent=0), @units.qual.BUC(unit="deg", exponent=0), @units.qual.BUC(unit="g", exponent=0), @units.qual.BUC(unit="hr", exponent=0), @units.qual.BUC(unit="m", exponent=0), @units.qual.BUC(unit="mol", exponent=0), @units.qual.BUC(unit="rad", exponent=-1), @units.qual.BUC(unit="s", exponent=0)}) int) (2)) * (@rad float) Math.atan2(sinHalfAngleAbs, cosHalfAngle);
        } else {
            hingeAngle = ((@UnitsRep(top=false, bot=false, prefixExponent=0, baseUnitComponents={@units.qual.BUC(unit="C", exponent=0), @units.qual.BUC(unit="K", exponent=0), @units.qual.BUC(unit="deg", exponent=0), @units.qual.BUC(unit="g", exponent=0), @units.qual.BUC(unit="hr", exponent=0), @units.qual.BUC(unit="m", exponent=0), @units.qual.BUC(unit="mol", exponent=0), @units.qual.BUC(unit="rad", exponent=-1), @units.qual.BUC(unit="s", exponent=0)}) int) (2)) * (@rad float) Math.atan2(sinHalfAngleAbs, -cosHalfAngle);
        }
        hingeAngle = computeNormalizedAngle(hingeAngle);
        return computeCorrespondingAngleNearLimits(hingeAngle, mLowerLimit, mUpperLimit);
    }

    /**
     * This structure is used to gather the information needed to create a hinge joint. This structure will be used to create the actual hinge joint.
     */
    public static class HingeJointInfo extends JointInfo {
        private final @Dimensionless Vector3 anchorPointWorldSpace;
        private final @Dimensionless Vector3 rotationAxisWorld;
        private final @Dimensionless boolean isLimitEnabled;
        private final @Dimensionless boolean isMotorEnabled;
        private final @Dimensionless float minAngleLimit;
        private final @Dimensionless float maxAngleLimit;
        private final @Dimensionless float motorSpeed;
        private final @Dimensionless float maxMotorTorque;

        /**
         * Constructs a new unlimited and non-motored hinge joint info from the both bodies, the initial anchor point in world space and the init axis, also in world space.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initRotationAxisWorld The initial axis in world space
         */
        public HingeJointInfo(HingeJoint.@Dimensionless HingeJointInfo this, @Dimensionless RigidBody body1, @Dimensionless RigidBody body2, @Dimensionless Vector3 initAnchorPointWorldSpace, @Dimensionless Vector3 initRotationAxisWorld) {
            super(body1, body2, JointType.HINGEJOINT);
            anchorPointWorldSpace = initAnchorPointWorldSpace;
            rotationAxisWorld = initRotationAxisWorld;
            isLimitEnabled = false;
            isMotorEnabled = false;
            minAngleLimit = ((@Dimensionless int) (-1));
            maxAngleLimit = ((@Dimensionless int) (1));
            motorSpeed = ((@Dimensionless int) (0));
            maxMotorTorque = ((@Dimensionless int) (0));
        }

        /**
         * Constructs a new limited but non-motored hinge joint info from the both bodies, the initial anchor point in world space, the initial axis, also in world space, and the upper and lower
         * limits of the joint.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initRotationAxisWorld The initial axis in world space
         * @param initMinAngleLimit The initial lower limit
         * @param initMaxAngleLimit The initial upper limit
         */
        public HingeJointInfo(HingeJoint.@Dimensionless HingeJointInfo this, @Dimensionless RigidBody body1, @Dimensionless RigidBody body2, @Dimensionless Vector3 initAnchorPointWorldSpace, @Dimensionless Vector3 initRotationAxisWorld, @Dimensionless float initMinAngleLimit, @Dimensionless float initMaxAngleLimit) {
            super(body1, body2, JointType.HINGEJOINT);
            anchorPointWorldSpace = initAnchorPointWorldSpace;
            rotationAxisWorld = initRotationAxisWorld;
            isLimitEnabled = true;
            isMotorEnabled = false;
            minAngleLimit = initMinAngleLimit;
            maxAngleLimit = initMaxAngleLimit;
            motorSpeed = ((@Dimensionless int) (0));
            maxMotorTorque = ((@Dimensionless int) (0));
        }

        /**
         * Constructs a new limited and motored hinge joint info from the both bodies, the initial anchor point in world space, the initial axis, also in world space, the upper and lower limits of the
         * joint, the motor speed and the maximum motor torque.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initRotationAxisWorld The initial axis in world space
         * @param initMinAngleLimit The initial lower limit
         * @param initMaxAngleLimit The initial upper limit
         * @param initMotorSpeed The initial motor speed
         * @param initMaxMotorTorque The initial maximum motor torque
         */
        public HingeJointInfo(HingeJoint.@Dimensionless HingeJointInfo this, @Dimensionless RigidBody body1, @Dimensionless RigidBody body2, @Dimensionless Vector3 initAnchorPointWorldSpace, @Dimensionless Vector3 initRotationAxisWorld, @Dimensionless float initMinAngleLimit, @Dimensionless float initMaxAngleLimit,
                              @Dimensionless
                              float initMotorSpeed, @Dimensionless float initMaxMotorTorque) {
            super(body1, body2, JointType.HINGEJOINT);
            anchorPointWorldSpace = initAnchorPointWorldSpace;
            rotationAxisWorld = initRotationAxisWorld;
            isLimitEnabled = true;
            isMotorEnabled = true;
            minAngleLimit = initMinAngleLimit;
            maxAngleLimit = initMaxAngleLimit;
            motorSpeed = initMotorSpeed;
            maxMotorTorque = initMaxMotorTorque;
        }

        /**
         * Returns the anchor point in world space.
         *
         * @return The anchor point in world space
         */
        public @Dimensionless Vector3 getAnchorPointWorldSpace(HingeJoint.@Dimensionless HingeJointInfo this) {
            return anchorPointWorldSpace;
        }

        /**
         * Returns the rotation axis in world space.
         *
         * @return The axis in world space
         */
        public @Dimensionless Vector3 getRotationAxisWorld(HingeJoint.@Dimensionless HingeJointInfo this) {
            return rotationAxisWorld;
        }

        /**
         * Returns true if the limits are active.
         *
         * @return Whether or not the limits are active
         */
        public @Dimensionless boolean isLimitEnabled(HingeJoint.@Dimensionless HingeJointInfo this) {
            return isLimitEnabled;
        }

        /**
         * Returns the lower angle limit.
         *
         * @return The lower limit
         */
        public @Dimensionless float getMinAngleLimit(HingeJoint.@Dimensionless HingeJointInfo this) {
            return minAngleLimit;
        }

        /**
         * Returns the upper angle limit.
         *
         * @return The upper limit
         */
        public @Dimensionless float getMaxAngleLimit(HingeJoint.@Dimensionless HingeJointInfo this) {
            return maxAngleLimit;
        }

        /**
         * Returns true if the motor is enabled.
         *
         * @return Whether or not the motor is enabled
         */
        public @Dimensionless boolean isMotorEnabled(HingeJoint.@Dimensionless HingeJointInfo this) {
            return isMotorEnabled;
        }

        /**
         * Returns the motor speed.
         *
         * @return The motor speed
         */
        public @Dimensionless float getMotorSpeed(HingeJoint.@Dimensionless HingeJointInfo this) {
            return motorSpeed;
        }

        /**
         * Returns the max motor torque.
         *
         * @return The motor torque
         */
        public @Dimensionless float getMaxMotorTorque(HingeJoint.@Dimensionless HingeJointInfo this) {
            return maxMotorTorque;
        }
    }
}
