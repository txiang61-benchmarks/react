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
import units.qual.ns;
import units.qual.Dimensionless;

/**
 * This class takes care of the time in the physics engine. It uses {@link System#nanoTime()} to get the current time.
 */
public class Timer {
    private @Dimensionless double mTimeStep;
    private @Dimensionless double mLastUpdateTime;
    private @Dimensionless double mDeltaTime;
    private @Dimensionless double mAccumulator;
    private @Dimensionless boolean mIsRunning = false;

    /**
     * Constructs a new timer from the time step.
     *
     * @param timeStep The time step
     */
    public Timer(double timeStep) {
        if (timeStep <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("time step cannot be smaller or equal to zero");
        }
        mTimeStep = timeStep;
    }

    /**
     * Gets the time step of the physics engine.
     *
     * @return The time step
     */
    public double getTimeStep(@Dimensionless Timer this) {
        return mTimeStep;
    }

    /**
     * Sets the time step of the physics engine.
     *
     * @param timeStep The time step to set
     */
    public void setTimeStep(@Dimensionless Timer this, @Dimensionless double timeStep) {
        if (timeStep <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("time step must be greater than zero");
        }
        mTimeStep = timeStep;
    }

    /**
     * Gets the current time.
     *
     * @return The current time
     */
    public double getPhysicsTime(@Dimensionless Timer this) {
        return mLastUpdateTime;
    }

    /**
     * Returns true if the timer is running, false if not.
     *
     * @return Whether or not the timer is running
     */
    public boolean isRunning(@Dimensionless Timer this) {
        return mIsRunning;
    }

    /**
     * Start the timer.
     */
    public void start(@Dimensionless Timer this) {
        if (!mIsRunning) {
            mLastUpdateTime = getCurrentSystemTime();
            mAccumulator = ((@Dimensionless int) (0));
            mIsRunning = true;
        }
    }

    /**
     * Stop the timer.
     */
    public void stop(@Dimensionless Timer this) {
        mIsRunning = false;
    }

    /**
     * Returns true if it's possible to take a new step, false if not.
     *
     * @return Whether or not a new step is possible
     */
    public boolean isPossibleToTakeStep(@Dimensionless Timer this) {
        return mAccumulator >= mTimeStep;
    }

    /**
     * Takes a new step: updates the timer by adding the timeStep value to the current time.
     */
    public void nextStep(@Dimensionless Timer this) {
        if (!mIsRunning) {
            throw new @Dimensionless IllegalStateException("Timer is not running");
        }
        mAccumulator -= mTimeStep;
    }

    /**
     * Compute the interpolation factor for the time step.
     *
     * @return The interpolation factor
     */
    public float computeInterpolationFactor(@Dimensionless Timer this) {
        return (@Dimensionless float) (mAccumulator / mTimeStep);
    }

    /**
     * Compute the time since the last update call and add it to the accumulator.
     */
    public void update(@Dimensionless Timer this) {
        final @Dimensionless double currentTime = getCurrentSystemTime();
        mDeltaTime = currentTime - mLastUpdateTime;
        mLastUpdateTime = currentTime;
        mAccumulator += mDeltaTime;
    }

    private static @Dimensionless double getCurrentSystemTime() {
        return System.nanoTime() / ((@ns double) (1e9d));
    }
}
