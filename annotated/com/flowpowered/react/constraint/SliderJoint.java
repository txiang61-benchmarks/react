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
import com.flowpowered.react.math.Mathematics;
import com.flowpowered.react.math.Matrix2x2;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Quaternion;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector2;
import com.flowpowered.react.math.Vector3;

/**
 * This class represents a slider joint. This joint has a one degree of freedom. It only allows relative translation of the bodies along a single direction and no rotation.
 */
public class SliderJoint extends Joint {
    private static final @Dimensionless float BETA = ((@Dimensionless float) (0.2f));
    private final @Dimensionless Vector3 mLocalAnchorPointBody1;
    private final @Dimensionless Vector3 mLocalAnchorPointBody2;
    private final @Dimensionless Vector3 mSliderAxisBody1;
    private final @Dimensionless Matrix3x3 mI1 = new @Dimensionless Matrix3x3();
    private final @Dimensionless Matrix3x3 mI2 = new @Dimensionless Matrix3x3();
    private final @Dimensionless Quaternion mInitOrientationDifferenceInv;
    private final @Dimensionless Vector3 mN1 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mN2 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR1 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR2 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR2CrossN1 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR2CrossN2 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR2CrossSliderAxis = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR1PlusUCrossN1 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR1PlusUCrossN2 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mR1PlusUCrossSliderAxis = new @Dimensionless Vector3();
    private final @Dimensionless Vector2 mBTranslation = new @Dimensionless Vector2();
    private final @Dimensionless Vector3 mBRotation = new @Dimensionless Vector3();
    private @Dimensionless float mBLowerLimit;
    private @Dimensionless float mBUpperLimit;
    private final @Dimensionless Matrix2x2 mInverseMassMatrixTranslationConstraint = new @Dimensionless Matrix2x2();
    private final @Dimensionless Matrix3x3 mInverseMassMatrixRotationConstraint = new @Dimensionless Matrix3x3();
    private @Dimensionless float mInverseMassMatrixLimit;
    private @Dimensionless float mInverseMassMatrixMotor;
    private final @Dimensionless Vector2 mImpulseTranslation;
    private final @Dimensionless Vector3 mImpulseRotation;
    private @Dimensionless float mImpulseLowerLimit;
    private @Dimensionless float mImpulseUpperLimit;
    private @Dimensionless float mImpulseMotor;
    private @Dimensionless boolean mIsLimitEnabled;
    private @Dimensionless boolean mIsMotorEnabled;
    private final @Dimensionless Vector3 mSliderAxisWorld = new @Dimensionless Vector3();
    private @Dimensionless float mLowerLimit;
    private @Dimensionless float mUpperLimit;
    private @Dimensionless boolean mIsLowerLimitViolated;
    private @Dimensionless boolean mIsUpperLimitViolated;
    private @Dimensionless float mMotorSpeed;
    private @Dimensionless float mMaxMotorForce;

    /**
     * Constructs a slider joint from provided slider joint info.
     *
     * @param jointInfo The joint info
     */
    public SliderJoint(SliderJointInfo jointInfo) {
        super(jointInfo);
        if (jointInfo.getMaxTranslationLimit() < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Upper limit must be greater or equal to 0");
        }
        if (jointInfo.getMinTranslationLimit() > ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Lower limit must be smaller or equal to 0");
        }
        if (jointInfo.getMaxMotorForce() < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Max motor force must be greater or equal to 0");
        }
        mImpulseTranslation = new @Dimensionless Vector2(((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mImpulseRotation = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mImpulseLowerLimit = ((@Dimensionless int) (0));
        mImpulseUpperLimit = ((@Dimensionless int) (0));
        mImpulseMotor = ((@Dimensionless int) (0));
        mIsLimitEnabled = jointInfo.isLimitEnabled();
        mIsMotorEnabled = jointInfo.isMotorEnabled();
        mLowerLimit = jointInfo.getMinTranslationLimit();
        mUpperLimit = jointInfo.getMaxTranslationLimit();
        mIsLowerLimitViolated = false;
        mIsUpperLimitViolated = false;
        mMotorSpeed = jointInfo.getMotorSpeed();
        mMaxMotorForce = jointInfo.getMaxMotorForce();
        final @Dimensionless Transform transform1 = mBody1.getTransform();
        final @Dimensionless Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = Transform.multiply(transform1.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mLocalAnchorPointBody2 = Transform.multiply(transform2.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mInitOrientationDifferenceInv = Quaternion.multiply(transform2.getOrientation(), transform1.getOrientation().getInverse());
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();
        mSliderAxisBody1 = Quaternion.multiply(mBody1.getTransform().getOrientation().getInverse(), jointInfo.getSliderAxisWorldSpace());
        mSliderAxisBody1.normalize();
    }

    @Override
    public void initBeforeSolve(@Dimensionless SliderJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        mIndexBody1 = constraintSolverData.getMapBodyToConstrainedVelocityIndex().get(mBody1);
        mIndexBody2 = constraintSolverData.getMapBodyToConstrainedVelocityIndex().get(mBody2);
        final @Dimensionless Vector3 x1 = mBody1.getTransform().getPosition();
        final @Dimensionless Vector3 x2 = mBody2.getTransform().getPosition();
        final @Dimensionless Quaternion orientationBody1 = mBody1.getTransform().getOrientation();
        final @Dimensionless Quaternion orientationBody2 = mBody2.getTransform().getOrientation();
        mI1.set(mBody1.getInertiaTensorInverseWorld());
        mI2.set(mBody2.getInertiaTensorInverseWorld());
        mR1.set(Quaternion.multiply(orientationBody1, mLocalAnchorPointBody1));
        mR2.set(Quaternion.multiply(orientationBody2, mLocalAnchorPointBody2));
        final @Dimensionless Vector3 u = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2), x1), mR1);
        mSliderAxisWorld.set(Quaternion.multiply(orientationBody1, mSliderAxisBody1));
        mSliderAxisWorld.normalize();
        mN1.set(mSliderAxisWorld.getOneUnitOrthogonalVector());
        mN2.set(mSliderAxisWorld.cross(mN1));
        final @Dimensionless float uDotSliderAxis = u.dot(mSliderAxisWorld);
        final @Dimensionless float lowerLimitError = uDotSliderAxis - mLowerLimit;
        final @Dimensionless float upperLimitError = mUpperLimit - uDotSliderAxis;
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
        mR2CrossN1.set(mR2.cross(mN1));
        mR2CrossN2.set(mR2.cross(mN2));
        mR2CrossSliderAxis.set(mR2.cross(mSliderAxisWorld));
        final @Dimensionless Vector3 r1PlusU = Vector3.add(mR1, u);
        mR1PlusUCrossN1.set(r1PlusU.cross(mN1));
        mR1PlusUCrossN2.set(r1PlusU.cross(mN2));
        mR1PlusUCrossSliderAxis.set(r1PlusU.cross(mSliderAxisWorld));
        @Dimensionless
        float sumInverseMass = ((@Dimensionless int) (0));
        final @Dimensionless Vector3 I1R1PlusUCrossN1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I1R1PlusUCrossN2 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I2R2CrossN1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I2R2CrossN2 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        if (mBody1.isMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1.set(Matrix3x3.multiply(mI1, mR1PlusUCrossN1));
            I1R1PlusUCrossN2.set(Matrix3x3.multiply(mI1, mR1PlusUCrossN2));
        }
        if (mBody2.isMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1.set(Matrix3x3.multiply(mI2, mR2CrossN1));
            I2R2CrossN2.set(Matrix3x3.multiply(mI2, mR2CrossN2));
        }
        final @Dimensionless float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
        final @Dimensionless float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
        final @Dimensionless float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
        final @Dimensionless float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
        final @Dimensionless Matrix2x2 matrixKTranslation = new @Dimensionless Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslationConstraint.set(matrixKTranslation.getInverse());
        }
        mBTranslation.setToZero();
        final @Dimensionless float biasFactor = BETA / constraintSolverData.getTimeStep();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation.setX(u.dot(mN1));
            mBTranslation.setY(u.dot(mN2));
            mBTranslation.multiply(biasFactor);
        }
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.set(mInverseMassMatrixRotationConstraint.getInverse());
        }
        mBRotation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            final @Dimensionless Quaternion currentOrientationDifference = Quaternion.multiply(orientationBody2, orientationBody1.getInverse());
            currentOrientationDifference.normalize();
            final @Dimensionless Quaternion qError = Quaternion.multiply(currentOrientationDifference, mInitOrientationDifferenceInv);
            mBRotation.set(Vector3.multiply(biasFactor * ((@Dimensionless int) (2)), qError.getVectorV()));
        }
        if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)) {
            mInverseMassMatrixLimit = ((@Dimensionless int) (0));
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixLimit += mBody1.getMassInverse() + mR1PlusUCrossSliderAxis.dot(Matrix3x3.multiply(mI1, mR1PlusUCrossSliderAxis));
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixLimit += mBody2.getMassInverse() + mR2CrossSliderAxis.dot(Matrix3x3.multiply(mI2, mR2CrossSliderAxis));
            }
            mInverseMassMatrixLimit = mInverseMassMatrixLimit > ((@Dimensionless int) (0)) ? ((@Dimensionless int) (1)) / mInverseMassMatrixLimit : ((@Dimensionless int) (0));
            mBLowerLimit = ((@Dimensionless int) (0));
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBLowerLimit = biasFactor * lowerLimitError;
            }
            mBUpperLimit = ((@Dimensionless int) (0));
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBUpperLimit = biasFactor * upperLimitError;
            }
        }
        if (mIsMotorEnabled) {
            mInverseMassMatrixMotor = ((@Dimensionless int) (0));
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixMotor += mBody1.getMassInverse();
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixMotor += mBody2.getMassInverse();
            }
            mInverseMassMatrixMotor = mInverseMassMatrixMotor > ((@Dimensionless int) (0)) ? ((@Dimensionless int) (1)) / mInverseMassMatrixMotor : ((@Dimensionless int) (0));
        }
        if (!constraintSolverData.isWarmStartingActive()) {
            mImpulseTranslation.setToZero();
            mImpulseRotation.setToZero();
            mImpulseLowerLimit = ((@Dimensionless int) (0));
            mImpulseUpperLimit = ((@Dimensionless int) (0));
            mImpulseMotor = ((@Dimensionless int) (0));
        }
    }

    @Override
    public void warmstart(@Dimensionless SliderJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        final @Dimensionless Vector3 v1 = constraintSolverData.getLinearVelocities()[mIndexBody1];
        final @Dimensionless Vector3 v2 = constraintSolverData.getLinearVelocities()[mIndexBody2];
        final @Dimensionless Vector3 w1 = constraintSolverData.getAngularVelocities()[mIndexBody1];
        final @Dimensionless Vector3 w2 = constraintSolverData.getAngularVelocities()[mIndexBody2];
        final @Dimensionless float inverseMassBody1 = mBody1.getMassInverse();
        final @Dimensionless float inverseMassBody2 = mBody2.getMassInverse();
        final @Dimensionless float impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
        final @Dimensionless Vector3 linearImpulseLimits = Vector3.multiply(impulseLimits, mSliderAxisWorld);
        final @Dimensionless Vector3 impulseMotor = Vector3.multiply(mImpulseMotor, mSliderAxisWorld);
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), mImpulseTranslation.getX()), Vector3.multiply(mN2, mImpulseTranslation.getY()));
            final @Dimensionless Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mR1PlusUCrossN1), mImpulseTranslation.getX()),
                    Vector3.multiply(mR1PlusUCrossN2, mImpulseTranslation.getY()));
            angularImpulseBody1.add(Vector3.negate(mImpulseRotation));
            linearImpulseBody1.add(linearImpulseLimits);
            angularImpulseBody1.add(Vector3.multiply(impulseLimits, mR1PlusUCrossSliderAxis));
            linearImpulseBody1.add(impulseMotor);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = Vector3.add(Vector3.multiply(mN1, mImpulseTranslation.getX()), Vector3.multiply(mN2, mImpulseTranslation.getY()));
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mR2CrossN1, mImpulseTranslation.getX()), Vector3.multiply(mR2CrossN2, mImpulseTranslation.getY()));
            angularImpulseBody2.add(mImpulseRotation);
            linearImpulseBody2.add(Vector3.negate(linearImpulseLimits));
            angularImpulseBody2.add(Vector3.multiply(-impulseLimits, mR2CrossSliderAxis));
            linearImpulseBody2.add(Vector3.negate(impulseMotor));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
    }

    @Override
    public void solveVelocityConstraint(@Dimensionless SliderJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
        final @Dimensionless Vector3 v1 = constraintSolverData.getLinearVelocities()[mIndexBody1];
        final @Dimensionless Vector3 v2 = constraintSolverData.getLinearVelocities()[mIndexBody2];
        final @Dimensionless Vector3 w1 = constraintSolverData.getAngularVelocities()[mIndexBody1];
        final @Dimensionless Vector3 w2 = constraintSolverData.getAngularVelocities()[mIndexBody2];
        final @Dimensionless float inverseMassBody1 = mBody1.getMassInverse();
        final @Dimensionless float inverseMassBody2 = mBody2.getMassInverse();
        final @Dimensionless float el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) + mN1.dot(v2) + w2.dot(mR2CrossN1);
        final @Dimensionless float el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) + mN2.dot(v2) + w2.dot(mR2CrossN2);
        final @Dimensionless Vector2 JvTranslation = new @Dimensionless Vector2(el1, el2);
        final @Dimensionless Vector2 deltaLambda = Matrix2x2.multiply(mInverseMassMatrixTranslationConstraint, Vector2.subtract(Vector2.negate(JvTranslation), mBTranslation));
        mImpulseTranslation.add(deltaLambda);
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), deltaLambda.getX()), Vector3.multiply(mN2, deltaLambda.getY()));
            final @Dimensionless Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mR1PlusUCrossN1), deltaLambda.getX()), Vector3.multiply(mR1PlusUCrossN2, deltaLambda.getY()));
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = Vector3.add(Vector3.multiply(mN1, deltaLambda.getX()), Vector3.multiply(mN2, deltaLambda.getY()));
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mR2CrossN1, deltaLambda.getX()), Vector3.multiply(mR2CrossN2, deltaLambda.getY()));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
        final @Dimensionless Vector3 JvRotation = Vector3.subtract(w2, w1);
        final @Dimensionless Vector3 deltaLambda2 = Matrix3x3.multiply(mInverseMassMatrixRotationConstraint, Vector3.subtract(Vector3.negate(JvRotation), mBRotation));
        mImpulseRotation.add(deltaLambda2);
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 angularImpulseBody1 = Vector3.negate(deltaLambda2);
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 angularImpulseBody2 = deltaLambda2;
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
        if (mIsLimitEnabled) {
            if (mIsLowerLimitViolated) {
                final @Dimensionless float JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2) - mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);
                @Dimensionless
                float deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit - mBLowerLimit);
                final @Dimensionless float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = Math.max(mImpulseLowerLimit + deltaLambdaLower, ((@Dimensionless int) (0)));
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;
                if (mBody1.isMotionEnabled()) {
                    final @Dimensionless Vector3 linearImpulseBody1 = Vector3.multiply(-deltaLambdaLower, mSliderAxisWorld);
                    final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(-deltaLambdaLower, mR1PlusUCrossSliderAxis);
                    v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
                    w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
                }
                if (mBody2.isMotionEnabled()) {
                    final @Dimensionless Vector3 linearImpulseBody2 = Vector3.multiply(deltaLambdaLower, mSliderAxisWorld);
                    final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(deltaLambdaLower, mR2CrossSliderAxis);
                    v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
                    w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
                }
            }
            if (mIsUpperLimitViolated) {
                final @Dimensionless float JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1) - mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);
                @Dimensionless
                float deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit - mBUpperLimit);
                final @Dimensionless float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = Math.max(mImpulseUpperLimit + deltaLambdaUpper, ((@Dimensionless int) (0)));
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;
                if (mBody1.isMotionEnabled()) {
                    final @Dimensionless Vector3 linearImpulseBody1 = Vector3.multiply(deltaLambdaUpper, mSliderAxisWorld);
                    final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(deltaLambdaUpper, mR1PlusUCrossSliderAxis);
                    v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
                    w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
                }
                if (mBody2.isMotionEnabled()) {
                    final @Dimensionless Vector3 linearImpulseBody2 = Vector3.multiply(-deltaLambdaUpper, mSliderAxisWorld);
                    final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(-deltaLambdaUpper, mR2CrossSliderAxis);
                    v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
                    w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
                }
            }
        }
        if (mIsMotorEnabled) {
            final @Dimensionless float JvMotor = mSliderAxisWorld.dot(v1) - mSliderAxisWorld.dot(v2);
            final @Dimensionless float maxMotorImpulse = mMaxMotorForce * constraintSolverData.getTimeStep();
            @Dimensionless
            float deltaLambdaMotor = mInverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
            final @Dimensionless float lambdaTemp = mImpulseMotor;
            mImpulseMotor = Mathematics.clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;
            if (mBody1.isMotionEnabled()) {
                final @Dimensionless Vector3 linearImpulseBody1 = Vector3.multiply(deltaLambdaMotor, mSliderAxisWorld);
                v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            }
            if (mBody2.isMotionEnabled()) {
                final @Dimensionless Vector3 linearImpulseBody2 = Vector3.multiply(-deltaLambdaMotor, mSliderAxisWorld);
                v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            }
        }
    }

    @Override
    public void solvePositionConstraint(@Dimensionless SliderJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {
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
        mR1.set(Quaternion.multiply(q1, mLocalAnchorPointBody1));
        mR2.set(Quaternion.multiply(q2, mLocalAnchorPointBody2));
        final @Dimensionless Vector3 u = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2), x1), mR1);
        mSliderAxisWorld.set(Quaternion.multiply(q1, mSliderAxisBody1));
        mSliderAxisWorld.normalize();
        mN1.set(mSliderAxisWorld.getOneUnitOrthogonalVector());
        mN2.set(mSliderAxisWorld.cross(mN1));
        final @Dimensionless float uDotSliderAxis = u.dot(mSliderAxisWorld);
        final @Dimensionless float lowerLimitError = uDotSliderAxis - mLowerLimit;
        final @Dimensionless float upperLimitError = mUpperLimit - uDotSliderAxis;
        mIsLowerLimitViolated = lowerLimitError <= ((@Dimensionless int) (0));
        mIsUpperLimitViolated = upperLimitError <= ((@Dimensionless int) (0));
        mR2CrossN1.set(mR2.cross(mN1));
        mR2CrossN2.set(mR2.cross(mN2));
        mR2CrossSliderAxis.set(mR2.cross(mSliderAxisWorld));
        final @Dimensionless Vector3 r1PlusU = Vector3.add(mR1, u);
        mR1PlusUCrossN1.set(r1PlusU.cross(mN1));
        mR1PlusUCrossN2.set(r1PlusU.cross(mN2));
        mR1PlusUCrossSliderAxis.set(r1PlusU.cross(mSliderAxisWorld));
        @Dimensionless
        float sumInverseMass = ((@Dimensionless int) (0));
        final @Dimensionless Vector3 I1R1PlusUCrossN1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I1R1PlusUCrossN2 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I2R2CrossN1 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 I2R2CrossN2 = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        if (mBody1.isMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1.set(Matrix3x3.multiply(mI1, mR1PlusUCrossN1));
            I1R1PlusUCrossN2.set(Matrix3x3.multiply(mI1, mR1PlusUCrossN2));
        }
        if (mBody2.isMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1.set(Matrix3x3.multiply(mI2, mR2CrossN1));
            I2R2CrossN2.set(Matrix3x3.multiply(mI2, mR2CrossN2));
        }
        final @Dimensionless float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
        final @Dimensionless float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
        final @Dimensionless float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
        final @Dimensionless float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
        final @Dimensionless Matrix2x2 matrixKTranslation = new @Dimensionless Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslationConstraint.set(matrixKTranslation.getInverse());
        }
        final @Dimensionless Vector2 translationError = new @Dimensionless Vector2(u.dot(mN1), u.dot(mN2));
        final @Dimensionless Vector2 lambdaTranslation = Matrix2x2.multiply(mInverseMassMatrixTranslationConstraint, Vector2.negate(translationError));
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), lambdaTranslation.getX()), Vector3.multiply(mN2, lambdaTranslation.getY()));
            final @Dimensionless Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mR1PlusUCrossN1), lambdaTranslation.getX()),
                    Vector3.multiply(mR1PlusUCrossN2, lambdaTranslation.getY()));
            final @Dimensionless Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
            final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            x1.add(v1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 linearImpulseBody2 = Vector3.add(Vector3.multiply(mN1, lambdaTranslation.getX()), Vector3.multiply(mN2, lambdaTranslation.getY()));
            final @Dimensionless Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mR2CrossN1, lambdaTranslation.getX()), Vector3.multiply(mR2CrossN2, lambdaTranslation.getY()));
            final @Dimensionless Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
            final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            x2.add(v2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w2), q2), ((@Dimensionless float) (0.5f))));
            q2.normalize();
        }
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.set(mInverseMassMatrixRotationConstraint.getInverse());
        }
        final @Dimensionless Quaternion currentOrientationDifference = Quaternion.multiply(q2, q1.getInverse());
        currentOrientationDifference.normalize();
        final @Dimensionless Quaternion qError = Quaternion.multiply(currentOrientationDifference, mInitOrientationDifferenceInv);
        final @Dimensionless Vector3 errorRotation = Vector3.multiply(((@Dimensionless int) (2)), qError.getVectorV());
        final @Dimensionless Vector3 lambdaRotation = Matrix3x3.multiply(mInverseMassMatrixRotationConstraint, Vector3.negate(errorRotation));
        if (mBody1.isMotionEnabled()) {
            final @Dimensionless Vector3 angularImpulseBody1 = Vector3.negate(lambdaRotation);
            final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {
            final @Dimensionless Vector3 angularImpulseBody2 = lambdaRotation;
            final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w2), q2), ((@Dimensionless float) (0.5f))));
            q2.normalize();
        }
        if (mIsLimitEnabled) {
            if (mIsLowerLimitViolated || mIsUpperLimitViolated) {
                mInverseMassMatrixLimit = ((@Dimensionless int) (0));
                if (mBody1.isMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody1.getMassInverse() + mR1PlusUCrossSliderAxis.dot(Matrix3x3.multiply(mI1, mR1PlusUCrossSliderAxis));
                }
                if (mBody2.isMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody2.getMassInverse() + mR2CrossSliderAxis.dot(Matrix3x3.multiply(mI2, mR2CrossSliderAxis));
                }
                mInverseMassMatrixLimit = mInverseMassMatrixLimit > ((@Dimensionless int) (0)) ? ((@Dimensionless int) (1)) / mInverseMassMatrixLimit : ((@Dimensionless int) (0));
            }
            if (mIsLowerLimitViolated) {
                final @Dimensionless float lambdaLowerLimit = mInverseMassMatrixLimit * -lowerLimitError;
                if (mBody1.isMotionEnabled()) {
                    final @Dimensionless Vector3 linearImpulseBody1 = Vector3.multiply(-lambdaLowerLimit, mSliderAxisWorld);
                    final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(-lambdaLowerLimit, mR1PlusUCrossSliderAxis);
                    final @Dimensionless Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
                    final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
                    x1.add(v1);
                    q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {
                    final @Dimensionless Vector3 linearImpulseBody2 = Vector3.multiply(lambdaLowerLimit, mSliderAxisWorld);
                    final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(lambdaLowerLimit, mR2CrossSliderAxis);
                    final @Dimensionless Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
                    final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
                    x2.add(v2);
                    q2.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w2), q2), ((@Dimensionless float) (0.5f))));
                    q2.normalize();
                }
            }
            if (mIsUpperLimitViolated) {
                final @Dimensionless float lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);
                if (mBody1.isMotionEnabled()) {
                    final @Dimensionless Vector3 linearImpulseBody1 = Vector3.multiply(lambdaUpperLimit, mSliderAxisWorld);
                    final @Dimensionless Vector3 angularImpulseBody1 = Vector3.multiply(lambdaUpperLimit, mR1PlusUCrossSliderAxis);
                    final @Dimensionless Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
                    final @Dimensionless Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
                    x1.add(v1);
                    q1.add(Quaternion.multiply(Quaternion.multiply(new @Dimensionless Quaternion(((@Dimensionless int) (0)), w1), q1), ((@Dimensionless float) (0.5f))));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {
                    final @Dimensionless Vector3 linearImpulseBody2 = Vector3.multiply(-lambdaUpperLimit, mSliderAxisWorld);
                    final @Dimensionless Vector3 angularImpulseBody2 = Vector3.multiply(-lambdaUpperLimit, mR2CrossSliderAxis);
                    final @Dimensionless Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
                    final @Dimensionless Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
                    x2.add(v2);
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
    public void enableLimit(@Dimensionless SliderJoint this, @Dimensionless boolean isLimitEnabled) {
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
    public void enableMotor(@Dimensionless SliderJoint this, @Dimensionless boolean isMotorEnabled) {
        mIsMotorEnabled = isMotorEnabled;
        mImpulseMotor = ((@Dimensionless int) (0));
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    /**
     * Sets the minimum translation limit.
     *
     * @param lowerLimit The minimum limit
     */
    public void setMinTranslationLimit(@Dimensionless SliderJoint this, @Dimensionless float lowerLimit) {
        if (lowerLimit > mUpperLimit) {
            throw new @Dimensionless IllegalArgumentException("Lower limit must be smaller or equal to current upper limit");
        }
        if (lowerLimit != mLowerLimit) {
            mLowerLimit = lowerLimit;
            resetLimits();
        }
    }

    /**
     * Sets the maximum translation limit.
     *
     * @param upperLimit The maximum limit
     */
    public void setMaxTranslationLimit(@Dimensionless SliderJoint this, @Dimensionless float upperLimit) {
        if (mLowerLimit > upperLimit) {
            throw new @Dimensionless IllegalArgumentException("Current lower limit must be smaller or equal to upper limit");
        }
        if (upperLimit != mUpperLimit) {
            mUpperLimit = upperLimit;
            resetLimits();
        }
    }

    // Resets the limits.
    private void resetLimits(@Dimensionless SliderJoint this) {
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
    public void setMotorSpeed(@Dimensionless SliderJoint this, @Dimensionless float motorSpeed) {
        if (motorSpeed != mMotorSpeed) {
            mMotorSpeed = motorSpeed;
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

    /**
     * Sets the maximum motor force.
     *
     * @param maxMotorForce The maximum motor force
     */
    public void setMaxMotorForce(@Dimensionless SliderJoint this, @Dimensionless float maxMotorForce) {
        if (mMaxMotorForce < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Max motor force must be greater or equal to 0");
        }
        if (maxMotorForce != mMaxMotorForce) {
            mMaxMotorForce = maxMotorForce;
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

    /**
     * Return true if the limits or the joint are enabled.
     *
     * @return Whether or not the limits are enabled
     */
    public @Dimensionless boolean isLimitEnabled(@Dimensionless SliderJoint this) {
        return mIsLimitEnabled;
    }

    /**
     * Returns true if the motor of the joint is enabled.
     *
     * @return Whether or not the motor is enabled
     */
    public @Dimensionless boolean isMotorEnabled(@Dimensionless SliderJoint this) {
        return mIsMotorEnabled;
    }

    /**
     * Returns the current translation value of the joint.
     *
     * @return The current translation
     */
    public @Dimensionless float getTranslation(@Dimensionless SliderJoint this) {
        final @Dimensionless Vector3 x1 = mBody1.getTransform().getPosition();
        final @Dimensionless Vector3 x2 = mBody2.getTransform().getPosition();
        final @Dimensionless Quaternion q1 = mBody1.getTransform().getOrientation();
        final @Dimensionless Quaternion q2 = mBody2.getTransform().getOrientation();
        final @Dimensionless Vector3 anchorBody1 = Vector3.add(x1, Quaternion.multiply(q1, mLocalAnchorPointBody1));
        final @Dimensionless Vector3 anchorBody2 = Vector3.add(x2, Quaternion.multiply(q2, mLocalAnchorPointBody2));
        final @Dimensionless Vector3 u = Vector3.subtract(anchorBody2, anchorBody1);
        final @Dimensionless Vector3 sliderAxisWorld = Quaternion.multiply(q1, mSliderAxisBody1);
        sliderAxisWorld.normalize();
        return u.dot(sliderAxisWorld);
    }

    /**
     * Returns the minimum limit.
     *
     * @return The minimum limit
     */
    public @Dimensionless float getMinTranslationLimit(@Dimensionless SliderJoint this) {
        return mLowerLimit;
    }

    /**
     * Returns the maximum limit.
     *
     * @return The maximum limit
     */
    public @Dimensionless float getMaxTranslationLimit(@Dimensionless SliderJoint this) {
        return mUpperLimit;
    }

    /**
     * Returns the motor speed.
     *
     * @return The motor speed
     */
    public @Dimensionless float getMotorSpeed(@Dimensionless SliderJoint this) {
        return mMotorSpeed;
    }

    /**
     * Returns the maximum motor force.
     *
     * @return The maximum motor force
     */
    public @Dimensionless float getMaxMotorForce(@Dimensionless SliderJoint this) {
        return mMaxMotorForce;
    }

    /**
     * Returns the intensity of the current force applied for the joint motor.
     *
     * @param timeStep The simulation time step
     * @return The motor force for the time step
     */
    public @Dimensionless float getMotorForce(@Dimensionless SliderJoint this, @Dimensionless float timeStep) {
        return mImpulseMotor / timeStep;
    }

    /**
     * This structure is used to gather the information needed to create a slider joint. This structure will be used to create the actual slider joint.
     */
    public static class SliderJointInfo extends JointInfo {
        private final @Dimensionless Vector3 anchorPointWorldSpace = new @Dimensionless Vector3();
        private final @Dimensionless Vector3 sliderAxisWorldSpace = new @Dimensionless Vector3();
        private final @Dimensionless boolean isLimitEnabled;
        private final @Dimensionless boolean isMotorEnabled;
        private final @Dimensionless float minTranslationLimit;
        private final @Dimensionless float maxTranslationLimit;
        private final @Dimensionless float motorSpeed;
        private final @Dimensionless float maxMotorForce;

        /**
         * Constructs a new unlimited and non-motored slider joint info from the both bodies, the initial anchor point in world space and the init axis, also in world space.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initSliderAxisWorldSpace The initial axis in world space
         */
        public SliderJointInfo(SliderJoint.@Dimensionless SliderJointInfo this, @Dimensionless RigidBody body1, @Dimensionless RigidBody body2, @Dimensionless Vector3 initAnchorPointWorldSpace, @Dimensionless Vector3 initSliderAxisWorldSpace) {
            super(body1, body2, JointType.SLIDERJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
            sliderAxisWorldSpace.set(initSliderAxisWorldSpace);
            isLimitEnabled = false;
            isMotorEnabled = false;
            minTranslationLimit = ((@Dimensionless int) (-1));
            maxTranslationLimit = ((@Dimensionless int) (1));
            motorSpeed = ((@Dimensionless int) (0));
            maxMotorForce = ((@Dimensionless int) (0));
        }

        /**
         * Constructs a new limited but non-motored slider joint info from the both bodies, the initial anchor point in world space, the initial axis, also in world space, and the upper and lower
         * limits of the joint.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initSliderAxisWorldSpace The initial axis in world space
         * @param initLowerLimit The initial lower limit
         * @param initUpperLimit The initial upper limit
         */
        public SliderJointInfo(SliderJoint.@Dimensionless SliderJointInfo this, @Dimensionless RigidBody body1, @Dimensionless RigidBody body2, @Dimensionless Vector3 initAnchorPointWorldSpace, @Dimensionless Vector3 initSliderAxisWorldSpace, @Dimensionless float initLowerLimit, @Dimensionless float initUpperLimit) {
            super(body1, body2, JointType.SLIDERJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
            sliderAxisWorldSpace.set(initSliderAxisWorldSpace);
            isLimitEnabled = true;
            isMotorEnabled = false;
            minTranslationLimit = initLowerLimit;
            maxTranslationLimit = initUpperLimit;
            motorSpeed = ((@Dimensionless int) (0));
            maxMotorForce = ((@Dimensionless int) (0));
        }

        /**
         * Constructs a new limited and motored slider joint info from the both bodies, the initial anchor point in world space, the initial axis, also in world space, the upper and lower limits of
         * the joint, the motor speed and the maximum motor force.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initSliderAxisWorldSpace The initial axis in world space
         * @param initLowerLimit The initial lower limit
         * @param initUpperLimit The initial upper limit
         * @param initMotorSpeed The initial motor speed
         * @param initMaxMotorForce The initial maximum motor force
         */
        public SliderJointInfo(SliderJoint.@Dimensionless SliderJointInfo this, @Dimensionless RigidBody body1, @Dimensionless RigidBody body2, @Dimensionless Vector3 initAnchorPointWorldSpace, @Dimensionless Vector3 initSliderAxisWorldSpace, @Dimensionless float initLowerLimit, @Dimensionless float initUpperLimit, @Dimensionless float initMotorSpeed,
                               @Dimensionless
                               float initMaxMotorForce) {
            super(body1, body2, JointType.SLIDERJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
            sliderAxisWorldSpace.set(initSliderAxisWorldSpace);
            isLimitEnabled = true;
            isMotorEnabled = true;
            minTranslationLimit = initLowerLimit;
            maxTranslationLimit = initUpperLimit;
            motorSpeed = initMotorSpeed;
            maxMotorForce = initMaxMotorForce;
        }

        /**
         * Returns the anchor point in world space.
         *
         * @return The anchor point in world space
         */
        public @Dimensionless Vector3 getAnchorPointWorldSpace(SliderJoint.@Dimensionless SliderJointInfo this) {
            return anchorPointWorldSpace;
        }

        /**
         * Returns the axis in world space.
         *
         * @return The axis in world space
         */
        public @Dimensionless Vector3 getSliderAxisWorldSpace(SliderJoint.@Dimensionless SliderJointInfo this) {
            return sliderAxisWorldSpace;
        }

        /**
         * Returns true if the limits are active.
         *
         * @return Whether or not the limits are active
         */
        public @Dimensionless boolean isLimitEnabled(SliderJoint.@Dimensionless SliderJointInfo this) {
            return isLimitEnabled;
        }

        /**
         * Returns the lower limit.
         *
         * @return The lower limit
         */
        public @Dimensionless float getMinTranslationLimit(SliderJoint.@Dimensionless SliderJointInfo this) {
            return minTranslationLimit;
        }

        /**
         * Returns the upper limit.
         *
         * @return The upper limit
         */
        public @Dimensionless float getMaxTranslationLimit(SliderJoint.@Dimensionless SliderJointInfo this) {
            return maxTranslationLimit;
        }

        /**
         * Returns true if the motor is enabled.
         *
         * @return Whether or not the motor is enabled
         */
        public @Dimensionless boolean isMotorEnabled(SliderJoint.@Dimensionless SliderJointInfo this) {
            return isMotorEnabled;
        }

        /**
         * Returns the motor speed.
         *
         * @return The motor speed
         */
        public @Dimensionless float getMotorSpeed(SliderJoint.@Dimensionless SliderJointInfo this) {
            return motorSpeed;
        }

        /**
         * Returns the max motor force.
         *
         * @return The motor force
         */
        public @Dimensionless float getMaxMotorForce(SliderJoint.@Dimensionless SliderJointInfo this) {
            return maxMotorForce;
        }
    }
}
