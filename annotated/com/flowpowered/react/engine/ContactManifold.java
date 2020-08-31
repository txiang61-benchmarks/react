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
import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.constraint.ContactPoint;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * Represents the set of contact points between two bodies. The contact manifold is implemented in a way to cache the contact points among the frames for better stability, following the "Contact
 * Generation" presentation by Erwin Coumans at the GDC 2010 conference (bullet.googlecode.com/files/GDC10_Coumans_Erwin_Contact.pdf). Some code from this class is based on the implementation of the
 * btPersistentManifold class from the Bullet physics engine (www.http://bulletphysics.org). The contacts between two bodies are added one after the other in the cache. When the cache is full, one
 * point needs to be removed. The idea is to keep the point with the deepest penetration depth and  producing the larger area (for a more stable contact manifold). The new added point is always kept.
 */
@Dimensionless
public class ContactManifold {
    public static final @Dimensionless int MAX_CONTACT_POINTS_IN_MANIFOLD = ((@Dimensionless int) (4));
    private final @Dimensionless CollisionBody mBody1;
    private final @Dimensionless CollisionBody mBody2;
    private final @Dimensionless ContactPoint @Dimensionless [] mContactPoints = new @Dimensionless ContactPoint @Dimensionless [MAX_CONTACT_POINTS_IN_MANIFOLD];
    private @Dimensionless int mNbContactPoints = ((@Dimensionless int) (0));
    private final @Dimensionless Vector3 mFrictionVector1 = new @Dimensionless Vector3();
    private final @Dimensionless Vector3 mFrictionVector2 = new @Dimensionless Vector3();
    private @Dimensionless float mFrictionImpulse1 = ((@Dimensionless int) (0));
    private @Dimensionless float mFrictionImpulse2 = ((@Dimensionless int) (0));
    private @Dimensionless float mFrictionTwistImpulse = ((@Dimensionless int) (0));
    private @Dimensionless boolean mIsAlreadyInIsland;

    /**
     * Constructs a new contact manifold from the first and second body.
     *
     * @param body1 The first body
     * @param body2 The second body
     */
    public ContactManifold(@Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        mBody1 = body1;
        mBody2 = body2;
        mIsAlreadyInIsland = false;
    }

    /**
     * Returns the first body in the manifold.
     *
     * @return The first body
     */
    public @Dimensionless CollisionBody getFirstBody(@Dimensionless ContactManifold this) {
        return mBody1;
    }

    /**
     * Returns the second body in the manifold.
     *
     * @return The second body
     */
    public @Dimensionless CollisionBody getSecondBody(@Dimensionless ContactManifold this) {
        return mBody2;
    }

    /**
     * Gets the number of contact points in the manifold.
     *
     * @return The number of contact points
     */
    public @Dimensionless int getNbContactPoints(@Dimensionless ContactManifold this) {
        return mNbContactPoints;
    }

    /**
     * Gets the first friction vector3 at the center of the contact manifold.
     *
     * @return The first friction vector
     */
    public @Dimensionless Vector3 getFirstFrictionVector(@Dimensionless ContactManifold this) {
        return mFrictionVector1;
    }

    /**
     * Sets the first friction vector3 at the center of the contact manifold.
     *
     * @param frictionVector1 The friction vector to set
     */
    public void setFirstFrictionVector(@Dimensionless ContactManifold this, @Dimensionless Vector3 frictionVector1) {
        mFrictionVector1.set(frictionVector1);
    }

    /**
     * Gets the second friction vector3 at the center of the contact manifold.
     *
     * @return The second friction vector
     */
    public @Dimensionless Vector3 getSecondFrictionVector(@Dimensionless ContactManifold this) {
        return mFrictionVector2;
    }

    /**
     * Sets the second friction vector3 at the center of the contact manifold.
     *
     * @param frictionVector2 The friction vector to set
     */
    public void setSecondFrictionVector(@Dimensionless ContactManifold this, @Dimensionless Vector3 frictionVector2) {
        mFrictionVector2.set(frictionVector2);
    }

    /**
     * Gets the accumulated impulse for the first friction.
     *
     * @return The accumulated impulse
     */
    public @Dimensionless float getFirstFrictionImpulse(@Dimensionless ContactManifold this) {
        return mFrictionImpulse1;
    }

    /**
     * Sets the accumulated impulse for the first friction.
     *
     * @param frictionImpulse1 The impulse to set
     */
    public void setFirstFrictionImpulse(@Dimensionless ContactManifold this, @Dimensionless float frictionImpulse1) {
        mFrictionImpulse1 = frictionImpulse1;
    }

    /**
     * Gets the accumulated impulse for the second friction.
     *
     * @return The accumulated impulse
     */
    public @Dimensionless float getSecondFrictionImpulse(@Dimensionless ContactManifold this) {
        return mFrictionImpulse2;
    }

    /**
     * Sets the accumulated impulse for the second friction.
     *
     * @param frictionImpulse2 The impulse to set
     */
    public void setSecondFrictionImpulse(@Dimensionless ContactManifold this, @Dimensionless float frictionImpulse2) {
        mFrictionImpulse2 = frictionImpulse2;
    }

    /**
     * Gets the accumulated impulse for the friction twist.
     *
     * @return The accumulated twist impulse
     */
    public @Dimensionless float getFrictionTwistImpulse(@Dimensionless ContactManifold this) {
        return mFrictionTwistImpulse;
    }

    /**
     * Sets the accumulated impulse for the friction twist.
     *
     * @param frictionTwistImpulse The twist impulse to set
     */
    public void setFrictionTwistImpulse(@Dimensionless ContactManifold this, @Dimensionless float frictionTwistImpulse) {
        mFrictionTwistImpulse = frictionTwistImpulse;
    }

    /**
     * Gets the contact point of the manifold at the desired index, which is greater or equal to zero and smaller than {@link #getNbContactPoints()}.
     *
     * @param index The index of the contact point
     * @return The contact point
     * @throws IllegalArgumentException If the index is smaller than zero or greater than the number of constraints, as defined by {@link #getNbContactPoints()}
     */
    public @Dimensionless ContactPoint getContactPoint(@Dimensionless ContactManifold this, @Dimensionless int index) {
        if (index < ((@Dimensionless int) (0)) || index >= mNbContactPoints) {
            throw new @Dimensionless IllegalArgumentException("index must be greater than zero and smaller than nbContatPoints");
        }
        return mContactPoints[index];
    }

    /**
     * Returns true if the contact manifold has already been added into an island.
     *
     * @return Whether or not the contact manifold is already in an island
     */
    public @Dimensionless boolean isAlreadyInIsland(@Dimensionless ContactManifold this) {
        return mIsAlreadyInIsland;
    }

    /**
     * Sets whether or not this contact manifold has already been added into an island.
     *
     * @param isAlreadyInIsland Whether or not the contact manifold is already in an island
     */
    public void setIsAlreadyInIsland(@Dimensionless ContactManifold this, @Dimensionless boolean isAlreadyInIsland) {
        mIsAlreadyInIsland = isAlreadyInIsland;
    }

    /**
     * Adds a contact point in the manifold.
     *
     * @param contact The contact point to add
     */
    public void addContactPoint(@Dimensionless ContactManifold this, @Dimensionless ContactPoint contact) {
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbContactPoints; i++) {
            final @Dimensionless float distance = Vector3.subtract(mContactPoints[i].getWorldPointOnFirstBody(),
                    contact.getWorldPointOnFirstBody()).lengthSquare();
            if (distance <= ReactDefaults.PERSISTENT_CONTACT_DIST_THRESHOLD * ReactDefaults.PERSISTENT_CONTACT_DIST_THRESHOLD) {
                return;
            }
        }
        if (mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD) {
            final @Dimensionless int indexMaxPenetration = getIndexOfDeepestPenetration(contact);
            final @Dimensionless int indexToRemove = getIndexToRemove(indexMaxPenetration, contact.getLocalPointOnFirstBody());
            removeContactPoint(indexToRemove);
        }
        mContactPoints[mNbContactPoints] = contact;
        mNbContactPoints++;
    }

    /**
     * Clears the contact manifold. Removes all contact points.
     */
    public void clear(@Dimensionless ContactManifold this) {
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbContactPoints; i++) {
            mContactPoints[i] = null;
        }
        mNbContactPoints = ((@Dimensionless int) (0));
    }

    // Removes a contact point from the manifold.
    private void removeContactPoint(@Dimensionless ContactManifold this, @Dimensionless int index) {
        if (index >= mNbContactPoints) {
            throw new @Dimensionless IllegalArgumentException("index must be smaller than nbContactPoints");
        }
        if (mNbContactPoints <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalStateException("nbContactPoints must be greater than zero");
        }
        mContactPoints[index] = null;
        if (index < mNbContactPoints - ((@Dimensionless int) (1))) {
            mContactPoints[index] = mContactPoints[mNbContactPoints - ((@Dimensionless int) (1))];
        }
        mNbContactPoints--;
    }

    /**
     * Updates the contact manifold. First the world space coordinates of the current contacts in the manifold are recomputed from the corresponding transforms for the bodies because they have moved.
     * Then we remove the contacts with a negative penetration depth (meaning that the bodies are not penetrating anymore) and with a distance between the contact points in the plane orthogonal to the
     * contact normal that is too large.
     *
     * @param transform1 The transform of the first body
     * @param transform2 The transform of the second body
     */
    public void update(@Dimensionless ContactManifold this, @Dimensionless Transform transform1, @Dimensionless Transform transform2) {
        if (mNbContactPoints == ((@Dimensionless int) (0))) {
            return;
        }
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbContactPoints; i++) {
            mContactPoints[i].setWorldPointOnFirstBody(Transform.multiply(transform1, mContactPoints[i].getLocalPointOnFirstBody()));
            mContactPoints[i].setWorldPointOnSecondBody(Transform.multiply(transform2, mContactPoints[i].getLocalPointOnSecondBody()));
            mContactPoints[i].setPenetrationDepth(Vector3.subtract(mContactPoints[i].getWorldPointOnFirstBody(), mContactPoints[i]
                    .getWorldPointOnSecondBody()).dot(mContactPoints[i].getNormal()));
        }
        final @Dimensionless float squarePersistentContactThreshold = ReactDefaults.PERSISTENT_CONTACT_DIST_THRESHOLD *
                ReactDefaults.PERSISTENT_CONTACT_DIST_THRESHOLD;
        for (@Dimensionless int i = mNbContactPoints - ((@Dimensionless int) (1)); i >= ((@Dimensionless int) (0)); i--) {
            if (i >= mNbContactPoints) {
                throw new @Dimensionless IllegalStateException("i must be smaller than nbContactPoints");
            }
            final @Dimensionless float distanceNormal = -mContactPoints[i].getPenetrationDepth();
            if (distanceNormal > squarePersistentContactThreshold) {
                removeContactPoint(i);
            } else {
                final @Dimensionless Vector3 projOfPoint1 = Vector3.add(
                        mContactPoints[i].getWorldPointOnFirstBody(),
                        Vector3.multiply(mContactPoints[i].getNormal(), distanceNormal));
                final @Dimensionless Vector3 projDifference = Vector3.subtract(mContactPoints[i].getWorldPointOnSecondBody(), projOfPoint1);
                if (projDifference.lengthSquare() > squarePersistentContactThreshold) {
                    removeContactPoint(i);
                }
            }
        }
    }

    // Returns the index of the contact point with the largest penetration depth.
    // This corresponding contact will be kept in the cache.
    // The method returns -1 if the new contact is the deepest.
    private @Dimensionless int getIndexOfDeepestPenetration(@Dimensionless ContactManifold this, @Dimensionless ContactPoint newContact) {
        if (mNbContactPoints != MAX_CONTACT_POINTS_IN_MANIFOLD) {
            throw new @Dimensionless IllegalStateException("nbContactPoints must be equal to MAX_CONTACT_POINTS_IN_MANIFOLD");
        }
        @Dimensionless
        int indexMaxPenetrationDepth = ((@Dimensionless int) (-1));
        @Dimensionless
        float maxPenetrationDepth = newContact.getPenetrationDepth();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbContactPoints; i++) {
            if (mContactPoints[i].getPenetrationDepth() > maxPenetrationDepth) {
                maxPenetrationDepth = mContactPoints[i].getPenetrationDepth();
                indexMaxPenetrationDepth = i;
            }
        }
        return indexMaxPenetrationDepth;
    }

    // Returns the index that will be removed.
    // The index of the contact point with the largest penetration depth is given as a parameter.
    // This contact won't be removed. Given this contact, we compute the different area; we want
    // to keep the contacts with the largest area. The new point is also kept.
    // In order to compute the area of a quadrilateral, we use the formula :
    // Area = 0.5 * ||AC x BD|| where AC and BD form the diagonals of the quadrilateral.
    // Note that when we compute this area, we do not calculate it exactly but only estimate it,
    // because we do not compute the actual diagonals of the quadrilateral.
    // Therefore, this is only a guess, which is faster to compute. This idea comes from the Bullet Physics library
    // by Erwin Coumans (http://wwww.bulletphysics.org).
    private @Dimensionless int getIndexToRemove(@Dimensionless ContactManifold this, @Dimensionless int indexMaxPenetration, @Dimensionless Vector3 newPoint) {
        if (mNbContactPoints != MAX_CONTACT_POINTS_IN_MANIFOLD) {
            throw new @Dimensionless IllegalStateException("nbContactPoints must be equal to MAX_CONTACT_POINTS_IN_MANIFOLD");
        }
        final @Dimensionless float area123N;
        final @Dimensionless float area023N;
        final @Dimensionless float area013N;
        final @Dimensionless float area012N;
        if (indexMaxPenetration != ((@Dimensionless int) (0))) {
            final @Dimensionless Vector3 vector1 = Vector3.subtract(newPoint, mContactPoints[((@Dimensionless int) (1))].getLocalPointOnFirstBody());
            final @Dimensionless Vector3 vector2 = Vector3.subtract(mContactPoints[((@Dimensionless int) (3))].getLocalPointOnFirstBody(), mContactPoints[((@Dimensionless int) (2))].getLocalPointOnFirstBody());
            final @Dimensionless Vector3 crossProduct = vector1.cross(vector2);
            area123N = crossProduct.lengthSquare();
        } else {
            area123N = ((@Dimensionless int) (0));
        }
        if (indexMaxPenetration != ((@Dimensionless int) (1))) {
            final @Dimensionless Vector3 vector1 = Vector3.subtract(newPoint, mContactPoints[((@Dimensionless int) (0))].getLocalPointOnFirstBody());
            final @Dimensionless Vector3 vector2 = Vector3.subtract(mContactPoints[((@Dimensionless int) (3))].getLocalPointOnFirstBody(), mContactPoints[((@Dimensionless int) (2))].getLocalPointOnFirstBody());
            final @Dimensionless Vector3 crossProduct = vector1.cross(vector2);
            area023N = crossProduct.lengthSquare();
        } else {
            area023N = ((@Dimensionless int) (1));
        }
        if (indexMaxPenetration != ((@Dimensionless int) (2))) {
            final @Dimensionless Vector3 vector1 = Vector3.subtract(newPoint, mContactPoints[((@Dimensionless int) (0))].getLocalPointOnFirstBody());
            final @Dimensionless Vector3 vector2 = Vector3.subtract(mContactPoints[((@Dimensionless int) (3))].getLocalPointOnFirstBody(), mContactPoints[((@Dimensionless int) (1))].getLocalPointOnFirstBody());
            final @Dimensionless Vector3 crossProduct = vector1.cross(vector2);
            area013N = crossProduct.lengthSquare();
        } else {
            area013N = ((@Dimensionless int) (2));
        }
        if (indexMaxPenetration != ((@Dimensionless int) (3))) {
            final @Dimensionless Vector3 vector1 = Vector3.subtract(newPoint, mContactPoints[((@Dimensionless int) (0))].getLocalPointOnFirstBody());
            final @Dimensionless Vector3 vector2 = Vector3.subtract(mContactPoints[((@Dimensionless int) (2))].getLocalPointOnFirstBody(), mContactPoints[((@Dimensionless int) (1))].getLocalPointOnFirstBody());
            final @Dimensionless Vector3 crossProduct = vector1.cross(vector2);
            area012N = crossProduct.lengthSquare();
        } else {
            area012N = ((@Dimensionless int) (3));
        }
        return getMaxArea(area123N, area023N, area013N, area012N);
    }

    // Returns the index of maximum area.
    private @Dimensionless int getMaxArea(@Dimensionless ContactManifold this, @Dimensionless float area123N, @Dimensionless float area023N, @Dimensionless float area013N, @Dimensionless float area012N) {
        if (area123N < area023N) {
            if (area023N < area013N) {
                if (area013N < area012N) {
                    return ((@Dimensionless int) (3));
                } else {
                    return ((@Dimensionless int) (2));
                }
            } else {
                if (area023N < area012N) {
                    return ((@Dimensionless int) (3));
                } else {
                    return ((@Dimensionless int) (1));
                }
            }
        } else {
            if (area123N < area013N) {
                if (area013N < area012N) {
                    return ((@Dimensionless int) (3));
                } else {
                    return ((@Dimensionless int) (2));
                }
            } else {
                if (area123N < area012N) {
                    return ((@Dimensionless int) (3));
                } else {
                    return ((@Dimensionless int) (0));
                }
            }
        }
    }

    /**
     * This structure represents a single element of a linked list of contact manifolds.
     */
    public static class ContactManifoldListElement {
        private @Dimensionless ContactManifold contactManifold;
        private @Dimensionless ContactManifoldListElement next;

        /**
         * Constructs a new contact manifold list element from the initial contact manifold and next list element.
         *
         * @param initContactManifold The contact manifold
         * @param initNext The next element
         */
        public ContactManifoldListElement(ContactManifold.@Dimensionless ContactManifoldListElement this, @Dimensionless ContactManifold initContactManifold, @Dimensionless ContactManifoldListElement initNext) {
            contactManifold = initContactManifold;
            next = initNext;
        }

        /**
         * Returns the contact manifold in this list element.
         *
         * @return The contact manifold
         */
        public @Dimensionless ContactManifold getContactManifold(ContactManifold.@Dimensionless ContactManifoldListElement this) {
            return contactManifold;
        }

        /**
         * Sets the contact manifold in this list element.
         *
         * @param contactManifold The contact manifold
         */
        public void setContactManifold(ContactManifold.@Dimensionless ContactManifoldListElement this, @Dimensionless ContactManifold contactManifold) {
            this.contactManifold = contactManifold;
        }

        /**
         * Returns the next element in the list.
         *
         * @return The next element
         */
        public @Dimensionless ContactManifoldListElement getNext(ContactManifold.@Dimensionless ContactManifoldListElement this) {
            return next;
        }

        /**
         * Sets the next element in the list.
         *
         * @param next The next element
         */
        public void setNext(ContactManifold.@Dimensionless ContactManifoldListElement this, @Dimensionless ContactManifoldListElement next) {
            this.next = next;
        }
    }
}
