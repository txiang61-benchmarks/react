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
package com.flowpowered.react.body;

import units.qual.Dimensionless;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.constraint.Joint;
import com.flowpowered.react.constraint.Joint.JointListElement;
import com.flowpowered.react.engine.Material;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a rigid body for the physics engine. A rigid body is a non-deformable body that has a constant mass. This class inherits from the CollisionBody class.
 */
public class RigidBody extends CollisionBody {
    private static final @Dimensionless Material DEFAULT_MATERIAL = Material.asUnmodifiableMaterial(new @Dimensionless Material());
    private @Dimensionless Material mMaterial = DEFAULT_MATERIAL;
    private @Dimensionless boolean mIsGravityEnabled;
    private @Dimensionless float mMass;
    private @Dimensionless float mMassInverse;
    private final @Dimensionless Matrix3x3 mInertiaTensorLocal = new @Dimensionless Matrix3x3();
    private final @Dimensionless Matrix3x3 mInertiaTensorLocalInverse;
    private final @Dimensionless Vector3 mExternalForce = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mExternalTorque = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mLinearVelocity = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mAngularVelocity = new @Dimensionless Vector3();
    private @Dimensionless float mLinearDamping;
    private @Dimensionless float mAngularDamping;
    private @Dimensionless JointListElement mJointsList;

    /**
     * Constructs a new rigid body from its transform, mass, local inertia tensor, collision shape and ID.
     *
     * @param transform The transform (position and orientation)
     * @param mass The mass
     * @param inertiaTensorLocal The local inertial tensor
     * @param collisionShape The collision shape
     * @param id The ID
     */
    public RigidBody(@Dimensionless Transform transform, @Dimensionless float mass, @Dimensionless Matrix3x3 inertiaTensorLocal, @Dimensionless CollisionShape collisionShape, @Dimensionless int id) {
        super(transform, collisionShape, id);
        mInertiaTensorLocal.set(inertiaTensorLocal);
        mMass = mass;
        mInertiaTensorLocalInverse = inertiaTensorLocal.getInverse();
        mMassInverse = ((@Dimensionless int) (1)) / mass;
        mIsGravityEnabled = true;
        mLinearDamping = ((@Dimensionless int) (0));
        mAngularDamping = ((@Dimensionless int) (0));
        mJointsList = null;
    }

    /**
     * Returns true if the gravity needs to be applied to this rigid body.
     *
     * @return Whether or not gravity should be applied to the body
     */
    public @Dimensionless boolean isGravityEnabled(@Dimensionless RigidBody this) {
        return mIsGravityEnabled;
    }

    /**
     * Sets the variable to know if the gravity is applied to this rigid body.
     *
     * @param isEnabled The new gravity state
     */
    public void enableGravity(@Dimensionless RigidBody this, @Dimensionless boolean isEnabled) {
        mIsGravityEnabled = isEnabled;
    }

    /**
     * Gets the mass of the body.
     *
     * @return The body's mass
     */
    public @Dimensionless float getMass(@Dimensionless RigidBody this) {
        return mMass;
    }

    /**
     * Sets the mass of the body.
     *
     * @param mass The mass to set
     */
    public void setMass(@Dimensionless RigidBody this, @Dimensionless float mass) {
        mMass = mass;
    }

    /**
     * Gets the inverse of the mass of the body.
     *
     * @return The inverse of the mass
     */
    public @Dimensionless float getMassInverse(@Dimensionless RigidBody this) {
        return mMassInverse;
    }

    /**
     * Sets the inverse of the mass.
     *
     * @param massInverse The inverse of the mass
     */
    public void setMassInverse(@Dimensionless RigidBody this, @Dimensionless float massInverse) {
        mMassInverse = massInverse;
    }

    /**
     * Gets the linear velocity of the body.
     *
     * @return The linear velocity
     */
    public @Dimensionless Vector3 getLinearVelocity(@Dimensionless RigidBody this) {
        return mLinearVelocity;
    }

    /**
     * Set the linear velocity for this body, but only if it can move.
     *
     * @param linearVelocity The linear velocity to set
     * @see #isMotionEnabled()
     * @see #enableMotion(boolean)
     */
    public void setLinearVelocity(@Dimensionless RigidBody this, @Dimensionless Vector3 linearVelocity) {
        if (mIsMotionEnabled) {
            mLinearVelocity.set(linearVelocity);
        }
    }

    /**
     * Gets the angular velocity of the body.
     *
     * @return The angular velocity
     */
    public @Dimensionless Vector3 getAngularVelocity(@Dimensionless RigidBody this) {
        return mAngularVelocity;
    }

    /**
     * Sets the angular velocity of the body.
     *
     * @param angularVelocity The angular velocity to set
     */
    public void setAngularVelocity(@Dimensionless RigidBody this, @Dimensionless Vector3 angularVelocity) {
        mAngularVelocity.set(angularVelocity);
    }

    /**
     * Gets the local inertia tensor of the body (in body coordinates).
     *
     * @return The local inertia tensor
     */
    public @Dimensionless Matrix3x3 getInertiaTensorLocal(@Dimensionless RigidBody this) {
        return mInertiaTensorLocal;
    }

    /**
     * Sets the local inertia tensor of the body (in body coordinates).
     *
     * @param inertiaTensorLocal The local inertia tensor to set
     */
    public void setInertiaTensorLocal(@Dimensionless RigidBody this, @Dimensionless Matrix3x3 inertiaTensorLocal) {
        mInertiaTensorLocal.set(inertiaTensorLocal);
    }

    /**
     * Gets the inertia tensor in world coordinates. The inertia tensor I_w in world coordinates is computed with the local inertia tensor I_b in body coordinates by I_w = R * I_b * R^T, where R is
     * the rotation matrix (and R^T its transpose) of the current orientation quaternion of the body.
     *
     * @return The world inertia tensor
     */
    public @Dimensionless Matrix3x3 getInertiaTensorWorld(@Dimensionless RigidBody this) {
        return Matrix3x3.multiply(Matrix3x3.multiply(mTransform.getOrientation().getMatrix(), mInertiaTensorLocal), mTransform.getOrientation().getMatrix().getTranspose());
    }

    /**
     * Gets the inverse of the inertia tensor in world coordinates. The inertia tensor I_w in world coordinates is computed with the local inverse inertia tensor I_b^-1 in body coordinates by I_w = R
     * * I_b^-1 * R^T, where R is the rotation matrix (and R^T its transpose) of the current orientation quaternion of the body.
     *
     * @return The world inverse inertia tensor
     */
    public @Dimensionless Matrix3x3 getInertiaTensorInverseWorld(@Dimensionless RigidBody this) {
        return Matrix3x3.multiply(Matrix3x3.multiply(mTransform.getOrientation().getMatrix(), mInertiaTensorLocalInverse), mTransform.getOrientation().getMatrix().getTranspose());
    }

    /**
     * Sets the rigid body's material.
     *
     * @param material The material to set
     */
    public void setMaterial(@Dimensionless RigidBody this, @Dimensionless Material material) {
        mMaterial = material;
    }

    /**
     * Gets the rigid body's material.
     *
     * @return The material
     */
    public @Dimensionless Material getMaterial(@Dimensionless RigidBody this) {
        return mMaterial;
    }

    /**
     * Returns the linear velocity damping factor.
     *
     * @return The linear damping
     */
    public @Dimensionless float getLinearDamping(@Dimensionless RigidBody this) {
        return mLinearDamping;
    }

    /**
     * Sets the linear damping factor.
     *
     * @param linearDamping The liner damping
     */
    public void setLinearDamping(@Dimensionless RigidBody this, @Dimensionless float linearDamping) {
        if (linearDamping < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Linear damping must be greater or equal to 0");
        }
        mLinearDamping = linearDamping;
    }

    /**
     * Returns the angular velocity damping factor.
     *
     * @return The angular damping
     */
    public @Dimensionless float getAngularDamping(@Dimensionless RigidBody this) {
        return mAngularDamping;
    }

    /**
     * Sets the angular damping factor.
     *
     * @param angularDamping The angular damping
     */
    public void setAngularDamping(@Dimensionless RigidBody this, @Dimensionless float angularDamping) {
        if (angularDamping < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Angular damping must be greater or equal to 0");
        }
        mAngularDamping = angularDamping;
    }

    /**
     * Returns the first element of the linked list of joints involving this body.
     *
     * @return The first element of the list
     */
    public @Dimensionless JointListElement getJointsList(@Dimensionless RigidBody this) {
        return mJointsList;
    }

    /**
     * Sets the first element in the joint list, discarding the entire list.
     *
     * @param jointsList The first element in the list
     */
    public void setJointsList(@Dimensionless RigidBody this, @Dimensionless JointListElement jointsList) {
        mJointsList = jointsList;
    }

    /**
     * Removes a joint from the joints list.
     *
     * @param joint The joint to remove
     */
    public void removeJointFromJointsList(@Dimensionless RigidBody this, @Dimensionless Joint joint) {
        if (joint == null) {
            throw new @Dimensionless IllegalArgumentException("Joint cannot be null");
        }
        if (mJointsList == null) {
            throw new @Dimensionless IllegalStateException("The joint list is already empty");
        }
        if (mJointsList.getJoint() == joint) {
            final @Dimensionless JointListElement elementToRemove = mJointsList;
            mJointsList = elementToRemove.getNext();
        } else {
            @Dimensionless
            JointListElement currentElement = mJointsList;
            while (currentElement.getNext() != null) {
                if (currentElement.getNext().getJoint() == joint) {
                    final @Dimensionless JointListElement elementToRemove = currentElement.getNext();
                    currentElement.setNext(elementToRemove.getNext());
                    break;
                }
                currentElement = currentElement.getNext();
            }
        }
    }

    @Override
    public void setIsSleeping(@Dimensionless RigidBody this, @Dimensionless boolean isSleeping) {
        if (isSleeping) {
            mLinearVelocity.setToZero();
            mAngularVelocity.setToZero();
            mExternalForce.setToZero();
            mExternalTorque.setToZero();
        }
        super.setIsSleeping(isSleeping);
    }

    /**
     * Applies an external force to the body at its gravity center. If the body is sleeping, calling this method will wake it up. Note that the force will be added to the sum of the applied forces and
     * that this sum will be reset to zero at the end of each call of the {@link com.flowpowered.react.engine.DynamicsWorld#update()} method.
     *
     * @param force The force to apply
     */
    public void applyForceToCenter(@Dimensionless RigidBody this, @Dimensionless Vector3 force) {
        if (!mIsMotionEnabled) {
            return;
        }
        if (mIsSleeping) {
            setIsSleeping(false);
        }
        mExternalForce.add(force);
    }

    /**
     * Applies an external force to the body at a given point (in world-space coordinates). If the point is not at the center of gravity of the body, it will also generate some torque and therefore, change
     * the angular velocity of the body. If the body is sleeping, calling this method will wake it up. Note that the force will be added to the sum of the applied forces and that this sum will be
     * reset to zero at the end of each call of the {@link com.flowpowered.react.engine.DynamicsWorld#update()} method.
     *
     * @param force The force to apply
     * @param point The point to apply the force to
     */
    public void applyForce(@Dimensionless RigidBody this, @Dimensionless Vector3 force, @Dimensionless Vector3 point) {
        if (!mIsMotionEnabled) {
            return;
        }
        if (mIsSleeping) {
            setIsSleeping(false);
        }
        mExternalForce.add(force);
        mExternalTorque.add(Vector3.subtract(point, mTransform.getPosition()).cross(force));
    }

    /**
     * Applies an external torque to the body. If the body is sleeping, calling this method will wake it up. Note that the force will be added to the sum of the applied torques and that this sum will
     * be reset to zero at the end of each call of the {@link com.flowpowered.react.engine.DynamicsWorld#update()} method.
     *
     * @param torque The torque to apply
     */
    public void applyTorque(@Dimensionless RigidBody this, @Dimensionless Vector3 torque) {

        // If it is a static body, do not apply any force
        if (!mIsMotionEnabled) {
            return;
        }

        // Awake the body if it was sleeping
        if (mIsSleeping) {
            setIsSleeping(false);
        }

        // Add the torque
        mExternalTorque.add(torque);
    }

    /**
     * Returns the total external force.
     *
     * @return The external force
     */
    public @Dimensionless Vector3 getExternalForce(@Dimensionless RigidBody this) {
        return mExternalForce;
    }

    /**
     * Returns the total external torque.
     *
     * @return The external torque
     */
    public @Dimensionless Vector3 getExternalTorque(@Dimensionless RigidBody this) {
        return mExternalTorque;
    }
}
