using System.Collections.Generic;
using UnityEngine;
using Modding;

public partial class RubberTireWheelScript
{
    public bool enableTireModel = true;
    public float muStatic = 1.40f;
    public float muKinetic = 1.20f;
    public float vStatic = 0.25f;
    public float forceFilterTau = 0.03f;

    public bool enableSinglePassLoadScaling = true;
    public bool enableCombinedSlipFriction = false;
    public float longitudinalGripScale = 1.0f;
    public float lateralGripScale = 1.0f;

    public bool enableModernLowSpeedTire = true;
    public float lowSpeedRelaxSpeedFloor = 1.0f;
    public float lowSpeedCreepSpeed = 0.15f;
    public float lowSpeedCreepBlend = 0.75f;
    public float lowSpeedShearDampingC = 1200f;
    public float axleAirDampingK = 2f;
    public float vEps = 0.2f;

    public bool enableTireRelaxation = true;
    public float relaxLength = 0.8f;
    public float shearK = 50000f;
    public float shearC = 0f;
    public float maxShearDisp = 0.15f;
    public bool resetShearOnNoContact = true;

    public bool decoupleTireForceAndTorque = true;
    public bool enableDecoupledTireForceApplication = false;

    // D1: scale grip by the ground collider's PhysicMaterial (ice, sand...).
    // Neutral reference is Unity's common 0.6 default so stock ground ≈ 1.0.
    public bool useSurfaceFriction = false;

    // D2: tyre load sensitivity, mu_eff = mu * (Fref/Fn)^s. 0 = off.
    public float muLoadSensitivity = 0f;
    public float muLoadReference = 2000f;

    private class TirePointState
    {
        public Vector3 shearDispWorld = Vector3.zero;
        public Vector3 FtireFiltered = Vector3.zero;
        public Vector3 lastPointWorld = Vector3.zero;
        public Vector3 lastNormalWorld = Vector3.up;
        public Vector3 lastConstraintSlipWorld = Vector3.zero;
        public Vector3 lastConstraintImpulseWorld = Vector3.zero;
        public bool hasConstraintHistory;
        public float lastFeedForwardForward;
        public float lastFeedForwardSide;
        public int lastSeenStep;
    }

    private readonly Dictionary<int, TirePointState> pointStates =
        new Dictionary<int, TirePointState>(16);
    private readonly List<int> pointStatesToRemove = new List<int>(16);

    private struct LateralPatchAccumulator
    {
        public int key;
        public Collider representativeCollider;
        public Rigidbody groundBody;
        public Vector3 weightedPoint;
        public Vector3 weightedNormal;
        public float normalLoad;
        public float legacyScaleLoad;
        public int debugIndex;
    }

    private readonly LateralPatchAccumulator[] lateralPatchBuffer =
        new LateralPatchAccumulator[8];
    private int lateralPatchCount;
    private float factoryTireLongForce;
    private float factoryTireLatForce;
    private float factoryTireNormalLoad;
    private float factoryTireLongSlip;
    private float factoryTireSideSlip;
    private float factoryStaticLockBlend;
    private bool factoryStaticConstraintSolved;

    // Global solver constants, not per-tyre tuning parameters.
    private const float StaticLockTimeConstant = 0.040f;
    private const float StaticStateMaxPointJump = 0.75f;
    private const float StaticStateMinNormalDot = 0.70f;

    // D1/D2: per-patch grip scale, set by ApplyAccumulatedLateralPatches before
    // that patch's force evaluation runs (ambient to avoid re-plumbing mu
    // through every solver helper).
    private const float SurfaceFrictionNeutral = 0.6f;
    private float activeMuScale = 1f;

    private float MuStaticEff() { return muStatic * activeMuScale; }
    private float MuKineticEff() { return muKinetic * activeMuScale; }

    private float ComputeMuScale(Collider groundCollider, float normalLoad)
    {
        float scale = 1f;
        if (useSurfaceFriction && groundCollider != null)
        {
            PhysicMaterial material = groundCollider.sharedMaterial;
            if (material != null)
                scale *= Mathf.Clamp(material.dynamicFriction / SurfaceFrictionNeutral, 0.05f, 2f);
        }
        float sensitivity = Mathf.Clamp(muLoadSensitivity, 0f, 0.5f);
        if (sensitivity > 1e-4f && normalLoad > 1e-3f)
        {
            float reference = Mathf.Max(1f, muLoadReference);
            scale *= Mathf.Clamp(Mathf.Pow(reference / normalLoad, sensitivity), 0.5f, 1.5f);
        }
        return scale;
    }

    private void ResetLateralPatchAccumulators()
    {
        lateralPatchCount = 0;
        factoryTireLongForce = 0f;
        factoryTireLatForce = 0f;
        factoryTireNormalLoad = 0f;
        factoryTireLongSlip = 0f;
        factoryTireSideSlip = 0f;
        factoryStaticLockBlend = 0f;
        factoryStaticConstraintSolved = false;
        activeMuScale = 1f;
    }

    private int AccumulateLateralPatch(
        ContactSample sample,
        Vector3 contactNormal,
        Rigidbody groundBody,
        float normalLoad,
        float legacyExtraScale)
    {
        // A dynamic body is one coupled contact island. Static colliders have
        // no shared Rigidbody, so keep them separate to avoid averaging a
        // floor and a neighbouring kerb into one fictitious plane.
        int key = groundBody != null
            ? groundBody.GetInstanceID()
            : (sample.col != null ? sample.col.GetInstanceID() : 0);
        int index = -1;
        for (int i = 0; i < lateralPatchCount; i++)
        {
            if (lateralPatchBuffer[i].key == key)
            {
                index = i;
                break;
            }
        }

        if (index < 0)
        {
            if (lateralPatchCount >= lateralPatchBuffer.Length) return -1;
            index = lateralPatchCount++;
            LateralPatchAccumulator created = new LateralPatchAccumulator();
            created.key = key;
            created.representativeCollider = sample.col;
            created.groundBody = groundBody;
            created.debugIndex = -1;
            lateralPatchBuffer[index] = created;
        }

        LateralPatchAccumulator patch = lateralPatchBuffer[index];
        patch.weightedPoint += sample.p * normalLoad;
        patch.weightedNormal += contactNormal * normalLoad;
        patch.normalLoad += normalLoad;
        patch.legacyScaleLoad += normalLoad * legacyExtraScale;
        lateralPatchBuffer[index] = patch;
        return index;
    }

    private void AttachLateralPatchDebugIndex(int patchIndex, int debugIndex)
    {
        if (patchIndex < 0 || patchIndex >= lateralPatchCount) return;
        LateralPatchAccumulator patch = lateralPatchBuffer[patchIndex];
        if (patch.debugIndex < 0) patch.debugIndex = debugIndex;
        lateralPatchBuffer[patchIndex] = patch;
    }

    private void ApplyAccumulatedLateralPatches(
        Vector3 wheelAxis,
        float fixedDeltaTime,
        bool updateDebug)
    {
        for (int i = 0; i < lateralPatchCount; i++)
        {
            LateralPatchAccumulator patch = lateralPatchBuffer[i];
            if (patch.normalLoad <= 1e-6f) continue;

            ContactSample sample = new ContactSample();
            sample.col = patch.representativeCollider;
            sample.colId = patch.representativeCollider != null
                ? patch.representativeCollider.GetInstanceID()
                : 0;
            sample.p = patch.weightedPoint / patch.normalLoad;
            sample.n = patch.weightedNormal;
            if (sample.n.sqrMagnitude > 1e-10f) sample.n.Normalize();
            else sample.n = Vector3.up;

            float legacyScale = patch.legacyScaleLoad / patch.normalLoad;
            activeMuScale = ComputeMuScale(sample.col, patch.normalLoad);
            Vector3 force = EvaluateAndApplyTireForce(
                patch.key,
                sample,
                sample.n,
                patch.groundBody,
                wheelAxis,
                patch.normalLoad,
                legacyScale,
                1f,
                fixedDeltaTime);

            Vector3 forward = ProjectOnPlane(Vector3.Cross(sample.n, wheelAxis), sample.n);
            if (forward.sqrMagnitude > 1e-8f) forward.Normalize();
            Vector3 side = ProjectOnPlane(wheelAxis, sample.n);
            if (side.sqrMagnitude > 1e-8f) side.Normalize();
            factoryTireLongForce += Vector3.Dot(force, forward);
            factoryTireLatForce += Vector3.Dot(force, side);
            factoryTireNormalLoad += patch.normalLoad;

            if (updateDebug
                && patch.debugIndex >= 0
                && patch.debugIndex < dbgLocalCount)
            {
                DebugContactLocalData debugData = dbgLocalContacts[patch.debugIndex];
                debugData.FtWorld = force;
                debugData.FtMag = force.magnitude;
                dbgLocalContacts[patch.debugIndex] = debugData;
            }
        }
    }

    private Vector3 EvaluateAndApplyTireForce(
        int patchKey,
        ContactSample sample,
        Vector3 contactNormal,
        Rigidbody groundBody,
        Vector3 wheelAxis,
        float normalLoad,
        float gate,
        float sampleWeight,
        float fixedDeltaTime)
    {
        Vector3 tireForce = Vector3.zero;
        if (!enableTireModel) return tireForce;

        Vector3 wheelPoint = GetNominalTreadPoint(sample.p, contactNormal, wheelAxis);
        Vector3 groundVelocity = groundBody != null
            ? groundBody.GetPointVelocity(sample.p)
            : Vector3.zero;
        Vector3 wheelVelocity = Rigidbody.GetPointVelocity(wheelPoint);
        Vector3 relativeVelocity = wheelVelocity - groundVelocity;

        Vector3 forward = ProjectOnPlane(Vector3.Cross(contactNormal, wheelAxis), contactNormal);
        if (forward.sqrMagnitude < 1e-6f)
            forward = ProjectOnPlane(Vector3.Cross(contactNormal, transform.right), contactNormal);
        if (forward.sqrMagnitude <= 1e-6f) return tireForce;
        forward.Normalize();

        Vector3 side = ProjectOnPlane(wheelAxis, contactNormal);
        if (side.sqrMagnitude < 1e-6f)
            side = ProjectOnPlane(Vector3.Cross(forward, contactNormal), contactNormal);
        if (side.sqrMagnitude > 1e-6f)
        {
            side.Normalize();
            if (Vector3.Dot(side, wheelAxis) < 0f) side = -side;
        }
        else
        {
            side = Vector3.zero;
        }

        Vector3 slipVelocity = ProjectOnPlane(relativeVelocity, contactNormal);
        float slipSpeed = slipVelocity.magnitude;
        factoryTireLongSlip = Vector3.Dot(slipVelocity, forward);
        factoryTireSideSlip = side.sqrMagnitude > 1e-8f
            ? Vector3.Dot(slipVelocity, side)
            : 0f;
        TirePointState state = GetOrCreatePatchState(
            patchKey,
            sample.p,
            contactNormal);
        state.lastSeenStep = fixedStepCounter;

        if (enableTireRelaxation)
        {
            if (enableSinglePassLoadScaling || enableCombinedSlipFriction || enableModernLowSpeedTire)
                state.shearDispWorld = ProjectOnPlane(state.shearDispWorld, contactNormal);

            float speedFloor = enableModernLowSpeedTire
                ? Mathf.Max(vEps, lowSpeedRelaxSpeedFloor)
                : vEps;
            float speed = Mathf.Max(slipSpeed, speedFloor);
            float relaxationTime = Mathf.Max(1e-4f, relaxLength / speed);

            float leakScale = 1f;
            if (enableModernLowSpeedTire)
            {
                float leakSpeed = Mathf.Max(1e-4f, lowSpeedCreepSpeed);
                leakScale = Mathf.Clamp01(slipSpeed / leakSpeed);
            }

            // Integrate x' = v - leak*x/tau analytically. The former explicit
            // Euler step becomes violently unstable when relaxLength / |slip|
            // is shorter than the physics step (for example 0.05 / 41.5 is
            // only 1.2 ms). The exact exponential update is passive for every
            // dt and cannot bounce the brush displacement across equilibrium.
            Vector3 previousShear = state.shearDispWorld;
            if (leakScale > 1e-5f)
            {
                float decayRate = leakScale / relaxationTime;
                float decay = Mathf.Exp(-decayRate * fixedDeltaTime);
                Vector3 equilibriumShear = slipVelocity / decayRate;
                state.shearDispWorld = equilibriumShear
                    + (previousShear - equilibriumShear) * decay;
            }
            else
            {
                state.shearDispWorld += slipVelocity * fixedDeltaTime;
            }
            Vector3 displacementRate = (state.shearDispWorld - previousShear)
                                     / Mathf.Max(1e-5f, fixedDeltaTime);

            if (maxShearDisp > 1e-5f)
            {
                float displacement = state.shearDispWorld.magnitude;
                if (displacement > maxShearDisp)
                    state.shearDispWorld *= maxShearDisp / displacement;
            }

            Vector3 rawForce = -shearK * state.shearDispWorld;
            float damping = shearC;
            if (enableModernLowSpeedTire)
                damping = Mathf.Max(damping, lowSpeedShearDampingC);
            if (damping > 0f) rawForce += -damping * displacementRate;

            bool staticZone = slipSpeed < Mathf.Max(1e-3f, vStatic);
            if (enableModernLowSpeedTire)
            {
                Vector3 creepForce = BuildLowSpeedCreepForce(slipVelocity, forward, side, normalLoad);
                float blendSpeed = Mathf.Max(1e-4f, lowSpeedCreepBlend);
                float creepBlend = 1f - Mathf.Clamp01(slipSpeed / blendSpeed);
                creepBlend = creepBlend * creepBlend * (3f - 2f * creepBlend);
                rawForce += creepForce * creepBlend;
            }

            if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
            {
                float longScale = Mathf.Max(0f, longitudinalGripScale);
                float latScale = Mathf.Max(0f, lateralGripScale);
                float scale;
                if (staticZone)
                {
                    scale = GetCombinedFrictionScale(
                        rawForce, forward, side,
                        MuStaticEff() * longScale * normalLoad,
                        MuStaticEff() * latScale * normalLoad);
                    if (scale < 0.9999f)
                        scale = GetCombinedFrictionScale(
                            rawForce, forward, side,
                            MuKineticEff() * longScale * normalLoad,
                            MuKineticEff() * latScale * normalLoad);
                }
                else
                {
                    scale = GetCombinedFrictionScale(
                        rawForce, forward, side,
                        MuKineticEff() * longScale * normalLoad,
                        MuKineticEff() * latScale * normalLoad);
                }

                if (scale < 0.9999f)
                {
                    rawForce *= scale;
                    state.shearDispWorld *= scale;
                }
            }
            else
            {
                float maximumStaticForce = MuStaticEff() * normalLoad;
                float maximumKineticForce = MuKineticEff() * normalLoad;
                float forceMagnitude = rawForce.magnitude;
                if (staticZone)
                {
                    if (forceMagnitude > maximumStaticForce && forceMagnitude > 1e-6f)
                    {
                        rawForce *= maximumKineticForce / forceMagnitude;
                        state.shearDispWorld = -rawForce / Mathf.Max(1e-6f, shearK);
                    }
                }
                else if (forceMagnitude > maximumKineticForce && forceMagnitude > 1e-6f)
                {
                    rawForce *= maximumKineticForce / forceMagnitude;
                    state.shearDispWorld = -rawForce / Mathf.Max(1e-6f, shearK);
                }
            }

            if (forceFilterTau > 1e-5f)
            {
                float filterAlpha = 1f - Mathf.Exp(
                    -fixedDeltaTime / Mathf.Max(1e-5f, forceFilterTau));
                state.FtireFiltered = Vector3.Lerp(state.FtireFiltered, rawForce, filterAlpha);
                tireForce = state.FtireFiltered;
            }
            else
            {
                state.FtireFiltered = rawForce;
                tireForce = rawForce;
            }

            if (enableSinglePassLoadScaling)
            {
                float postMu = staticZone ? MuStaticEff() : MuKineticEff();
                if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
                {
                    tireForce = ClampCombinedTireForce(
                        tireForce, forward, side,
                        postMu * Mathf.Max(0f, longitudinalGripScale) * normalLoad,
                        postMu * Mathf.Max(0f, lateralGripScale) * normalLoad);
                }
                else
                {
                    tireForce = LimitVectorMagnitude(tireForce, postMu * normalLoad);
                }
            }
        }
        else if (slipSpeed > 1e-5f)
        {
            if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
            {
                tireForce = BuildCombinedKineticFriction(
                    slipVelocity, forward, side,
                    MuKineticEff() * Mathf.Max(0f, longitudinalGripScale) * normalLoad,
                    MuKineticEff() * Mathf.Max(0f, lateralGripScale) * normalLoad);
            }
            else
            {
                tireForce = -slipVelocity / slipSpeed * (MuKineticEff() * normalLoad);
            }
        }

        if (!enableSinglePassLoadScaling)
        {
            tireForce *= gate;
            tireForce *= sampleWeight;
        }

        return ApplyLowSpeedStaticConstraint(
            state,
            tireForce,
            slipVelocity,
            forward,
            side,
            contactNormal,
            wheelPoint,
            sample.p,
            groundBody,
            wheelAxis,
            normalLoad,
            fixedDeltaTime);
    }

    private Vector3 GetNominalTreadPoint(
        Vector3 groundPoint,
        Vector3 contactNormal,
        Vector3 wheelAxis)
    {
        Vector3 center = treadPhysicsGeometryCached
            ? Rigidbody.position + Rigidbody.rotation * treadCenterBodyLocal
            : Rigidbody.worldCenterOfMass;
        float radius = treadPhysicsGeometryCached
            ? Mathf.Max(1e-4f, treadRadiusWorld)
            : Mathf.Max(1e-4f, Vector3.Distance(center, groundPoint));

        Vector3 radial = ProjectOnPlane(groundPoint - center, wheelAxis);
        if (radial.sqrMagnitude <= 1e-8f)
            radial = ProjectOnPlane(-contactNormal, wheelAxis);
        if (radial.sqrMagnitude <= 1e-8f)
            return groundPoint;
        radial.Normalize();
        if (Vector3.Dot(radial, -contactNormal) < 0f) radial = -radial;
        return center + radial * radius;
    }

    private Vector3 ApplyLowSpeedStaticConstraint(
        TirePointState state,
        Vector3 dynamicForce,
        Vector3 slipVelocity,
        Vector3 forward,
        Vector3 side,
        Vector3 contactNormal,
        Vector3 wheelPoint,
        Vector3 groundPoint,
        Rigidbody groundBody,
        Vector3 wheelAxis,
        float normalLoad,
        float fixedDeltaTime)
    {
        // Static friction is a contact-patch state, not a vehicle-speed state.
        // Pure rolling can stick at any road speed; using Rigidbody.velocity
        // here created a hard low-speed band where the bespoke constraint
        // replaced the working brush model, then vanished around 1 m/s.
        float slipForward = Vector3.Dot(slipVelocity, forward);
        float slipSide = side.sqrMagnitude > 1e-8f
            ? Vector3.Dot(slipVelocity, side)
            : 0f;
        float constraintSlipSpeed = Mathf.Sqrt(
            slipForward * slipForward + slipSide * slipSide);

        float staticFullSpeed = Mathf.Max(1e-3f, vStatic);
        float staticOffSpeed = Mathf.Max(staticFullSpeed + 0.05f, staticFullSpeed * 4f);
        float lockBlend = 1f - Mathf.Clamp01(
            (constraintSlipSpeed - staticFullSpeed)
            / Mathf.Max(1e-4f, staticOffSpeed - staticFullSpeed));
        lockBlend = lockBlend * lockBlend * (3f - 2f * lockBlend);
        factoryStaticLockBlend = lockBlend;
        factoryStaticConstraintSolved = false;

        if (!enableModernLowSpeedTire)
        {
            ResetStaticConstraintHistory(state);
            if (dynamicForce.sqrMagnitude > 1e-10f)
                ApplyTireForce(dynamicForce, wheelPoint, groundPoint, groundBody, wheelAxis);
            return dynamicForce;
        }

        if (lockBlend <= 1e-4f)
        {
            ResetStaticConstraintHistory(state);
            if (dynamicForce.sqrMagnitude > 1e-10f)
                ApplyTireForce(dynamicForce, wheelPoint, groundPoint, groundBody, wheelAxis);
            return dynamicForce;
        }

        // B1: feed the self-applied propulsion torque of THIS step forward so
        // the solver does not wait a step for the disturbance estimator.
        float feedForwardForward = 0f;
        float feedForwardSide = 0f;
        if (Mathf.Abs(pendingDriveAxisTorque) > 1e-6f)
        {
            Vector3 driveAngularImpulse = pendingDriveAxisWorld
                * (pendingDriveAxisTorque * fixedDeltaTime);
            Vector3 omegaDelta = MultiplyWorldInverseInertia(Rigidbody, driveAngularImpulse);
            Vector3 slipDelta = Vector3.Cross(
                omegaDelta,
                wheelPoint - Rigidbody.worldCenterOfMass);
            feedForwardForward = Vector3.Dot(slipDelta, forward);
            feedForwardSide = Vector3.Dot(slipDelta, side);
        }

        Vector3 staticImpulse;
        bool canStick = TrySolveStaticImpulse(
            state,
            slipVelocity,
            forward,
            side,
            wheelPoint,
            groundPoint,
            groundBody,
            normalLoad,
            fixedDeltaTime,
            feedForwardForward,
            feedForwardSide,
            out staticImpulse);

        if (!canStick)
        {
            ResetStaticConstraintHistory(state);
            // AddTorque is integrated after scripts, so dynamicForce only sees
            // the OLD slip. Include this step's known drive disturbance; when
            // sticking is overloaded, transmit kinetic-limit traction now
            // instead of returning zero for one frame and letting wheel speed
            // overshoot/oscillate.
            Vector3 predictedSlip = slipVelocity
                + forward * feedForwardForward
                + side * feedForwardSide;
            Vector3 fallbackForce = dynamicForce;
            if (predictedSlip.sqrMagnitude > 1e-10f)
            {
                fallbackForce = BuildKineticTireForce(
                    predictedSlip, forward, side, normalLoad);
            }
            if (fallbackForce.sqrMagnitude > 1e-10f)
                ApplyTireForce(fallbackForce, wheelPoint, groundPoint, groundBody, wheelAxis);
            return fallbackForce;
        }
        factoryStaticConstraintSolved = true;

        Vector3 dynamicImpulse = dynamicForce * fixedDeltaTime;
        Vector3 blendedImpulse = Vector3.Lerp(dynamicImpulse, staticImpulse, lockBlend);
        blendedImpulse = ClampStaticImpulse(
            blendedImpulse,
            forward,
            side,
            normalLoad * fixedDeltaTime);
        if (blendedImpulse.sqrMagnitude > 1e-12f)
            ApplyTireImpulse(blendedImpulse, wheelPoint, groundPoint, groundBody, wheelAxis);
        state.lastConstraintSlipWorld = slipVelocity;
        state.lastConstraintImpulseWorld = blendedImpulse;
        state.lastFeedForwardForward = feedForwardForward;
        state.lastFeedForwardSide = feedForwardSide;
        state.hasConstraintHistory = true;

        // B2: keep the brush state consistent with the applied tangential
        // impulse so the handoff back to the dynamic branch is continuous.
        Vector3 appliedForce = blendedImpulse / Mathf.Max(1e-5f, fixedDeltaTime);
        if (enableTireRelaxation && shearK > 1e-6f)
        {
            state.shearDispWorld = -appliedForce / shearK;
            if (maxShearDisp > 1e-5f)
            {
                float displacement = state.shearDispWorld.magnitude;
                if (displacement > maxShearDisp)
                    state.shearDispWorld *= maxShearDisp / displacement;
            }
            state.FtireFiltered = appliedForce;
        }
        return appliedForce;
    }

    private bool TrySolveStaticImpulse(
        TirePointState state,
        Vector3 slipVelocity,
        Vector3 forward,
        Vector3 side,
        Vector3 wheelPoint,
        Vector3 groundPoint,
        Rigidbody groundBody,
        float normalLoad,
        float fixedDeltaTime,
        float feedForwardForward,
        float feedForwardSide,
        out Vector3 impulseWorld)
    {
        impulseWorld = Vector3.zero;
        if (normalLoad <= 1e-6f) return false;
        if (side.sqrMagnitude <= 1e-8f) return false;

        float kFF = GetRelativePointVelocityResponse(
            forward, forward, wheelPoint, groundPoint, groundBody);
        float kFS = GetRelativePointVelocityResponse(
            forward, side, wheelPoint, groundPoint, groundBody);
        float kSF = GetRelativePointVelocityResponse(
            side, forward, wheelPoint, groundPoint, groundBody);
        // The analytical effective-mass matrix is symmetric. Averaging the
        // cross terms suppresses small floating-point asymmetry and keeps the
        // velocity correction passive.
        float kCross = 0.5f * (kFS + kSF);
        kFS = kCross;
        kSF = kCross;
        float kSS = GetRelativePointVelocityResponse(
            side, side, wheelPoint, groundPoint, groundBody);
        float determinant = kFF * kSS - kFS * kSF;
        if (determinant <= 1e-8f) return false;

        float dt = Mathf.Max(1e-5f, fixedDeltaTime);
        float velocityForward = Vector3.Dot(slipVelocity, forward);
        float velocitySide = Vector3.Dot(slipVelocity, side);

        // Estimate the unobserved per-step disturbance from the previous
        // velocity transition. This supplies the sustained reaction needed
        // for true sticking under gravity or chassis loads without blindly
        // reapplying an old impulse after that load disappears.
        float disturbanceForward = 0f;
        float disturbanceSide = 0f;
        if (state.hasConstraintHistory)
        {
            float previousVelocityForward = Vector3.Dot(
                state.lastConstraintSlipWorld,
                forward);
            float previousVelocitySide = Vector3.Dot(
                state.lastConstraintSlipWorld,
                side);
            float previousImpulseForward = Vector3.Dot(
                state.lastConstraintImpulseWorld,
                forward);
            float previousImpulseSide = Vector3.Dot(
                state.lastConstraintImpulseWorld,
                side);
            disturbanceForward = velocityForward
                               - previousVelocityForward
                               - kFF * previousImpulseForward
                               - kFS * previousImpulseSide
                               - state.lastFeedForwardForward;
            disturbanceSide = velocitySide
                            - previousVelocitySide
                            - kSF * previousImpulseForward
                            - kSS * previousImpulseSide
                            - state.lastFeedForwardSide;
        }

        float correction = 1f - Mathf.Exp(-dt / StaticLockTimeConstant);
        float inverseDeterminant = 1f / determinant;
        float targetForward = correction * velocityForward + disturbanceForward + feedForwardForward;
        float targetSide = correction * velocitySide + disturbanceSide + feedForwardSide;
        float deltaForward = -(kSS * targetForward - kFS * targetSide)
                           * inverseDeterminant;
        float deltaSide = -(-kSF * targetForward + kFF * targetSide)
                        * inverseDeterminant;

        float candidateForward = deltaForward;
        float candidateSide = deltaSide;
        float normalImpulse = normalLoad * dt;
        float maximumForward = MuStaticEff()
                             * (enableCombinedSlipFriction
                                ? Mathf.Max(0f, longitudinalGripScale)
                                : 1f)
                             * normalImpulse;
        float maximumSide = MuStaticEff()
                          * (enableCombinedSlipFriction
                             ? Mathf.Max(0f, lateralGripScale)
                             : 1f)
                          * normalImpulse;

        float usage = GetEllipseUsage(
            candidateForward,
            candidateSide,
            maximumForward,
            maximumSide);
        if (usage > 1f)
        {
            if (!state.hasConstraintHistory) return false;

            // A near-limit static load may leave no friction budget for
            // removing all residual velocity in one step. Stay in the static
            // branch when the estimated sustaining reaction itself fits, and
            // use the rim of the ellipse to arrest the residual over time.
            float sustainForward = -(kSS * disturbanceForward
                                   - kFS * disturbanceSide)
                                 * inverseDeterminant;
            float sustainSide = -(-kSF * disturbanceForward
                                + kFF * disturbanceSide)
                              * inverseDeterminant;
            float sustainUsage = GetEllipseUsage(
                sustainForward,
                sustainSide,
                maximumForward,
                maximumSide);
            if (sustainUsage > 1f) return false;

            float scale = 1f / Mathf.Sqrt(usage);
            candidateForward *= scale;
            candidateSide *= scale;
        }

        impulseWorld = forward * candidateForward + side * candidateSide;
        return true;
    }

    private void ResetStaticConstraintHistory(TirePointState state)
    {
        state.lastConstraintSlipWorld = Vector3.zero;
        state.lastConstraintImpulseWorld = Vector3.zero;
        state.lastFeedForwardForward = 0f;
        state.lastFeedForwardSide = 0f;
        state.hasConstraintHistory = false;
    }

    private float GetRelativePointVelocityResponse(
        Vector3 measureAxis,
        Vector3 impulseAxis,
        Vector3 wheelPoint,
        Vector3 groundPoint,
        Rigidbody groundBody)
    {
        Vector3 response = GetPointVelocityChange(Rigidbody, wheelPoint, impulseAxis);
        if (groundBody != null && groundBody != Rigidbody)
            response += GetPointVelocityChange(groundBody, groundPoint, impulseAxis);
        return Vector3.Dot(measureAxis, response);
    }

    private Vector3 GetPointVelocityChange(
        Rigidbody body,
        Vector3 point,
        Vector3 impulse)
    {
        if (body == null || body.isKinematic) return Vector3.zero;

        Vector3 linear = body.mass > 1e-6f
            ? impulse / body.mass
            : Vector3.zero;
        Vector3 arm = point - body.worldCenterOfMass;
        Vector3 angularImpulse = Vector3.Cross(arm, impulse);
        Vector3 angularVelocity = MultiplyWorldInverseInertia(body, angularImpulse);
        return linear + Vector3.Cross(angularVelocity, arm);
    }

    private Vector3 ClampStaticImpulse(
        Vector3 impulse,
        Vector3 forward,
        Vector3 side,
        float normalImpulse)
    {
        float maximumForward = MuStaticEff()
                             * (enableCombinedSlipFriction
                                ? Mathf.Max(0f, longitudinalGripScale)
                                : 1f)
                             * normalImpulse;
        float maximumSide = MuStaticEff()
                          * (enableCombinedSlipFriction
                             ? Mathf.Max(0f, lateralGripScale)
                             : 1f)
                          * normalImpulse;
        float forwardImpulse = Vector3.Dot(impulse, forward);
        float sideImpulse = Vector3.Dot(impulse, side);
        float usage = GetEllipseUsage(
            forwardImpulse,
            sideImpulse,
            maximumForward,
            maximumSide);
        if (usage > 1f)
        {
            float scale = 1f / Mathf.Sqrt(usage);
            forwardImpulse *= scale;
            sideImpulse *= scale;
        }
        return forward * forwardImpulse + side * sideImpulse;
    }

    private float GetEllipseUsage(
        float forwardValue,
        float sideValue,
        float maximumForward,
        float maximumSide)
    {
        if (maximumForward <= 1e-8f || maximumSide <= 1e-8f)
            return float.PositiveInfinity;
        return forwardValue * forwardValue / (maximumForward * maximumForward)
             + sideValue * sideValue / (maximumSide * maximumSide);
    }

    private void ApplyTireForce(
        Vector3 force,
        Vector3 wheelPoint,
        Vector3 groundPoint,
        Rigidbody groundRb,
        Vector3 wheelAxisWorld)
    {
        if (force.sqrMagnitude <= 1e-12f) return;

        bool useDecoupled = enableDecoupledTireForceApplication && decoupleTireForceAndTorque;
        if (useDecoupled)
        {
            Rigidbody.AddForce(force, ForceMode.Force);
            if (wheelAxisWorld.sqrMagnitude > 1e-10f)
            {
                wheelAxisWorld.Normalize();
                Vector3 tau = Vector3.Cross(wheelPoint - Rigidbody.worldCenterOfMass, force);
                float spinTau = Vector3.Dot(tau, wheelAxisWorld);
                if (Mathf.Abs(spinTau) > 1e-6f)
                    Rigidbody.AddTorque(wheelAxisWorld * spinTau, ForceMode.Force);
            }
        }
        else
        {
            Rigidbody.AddForceAtPosition(force, wheelPoint, ForceMode.Force);
        }

        if (groundRb != null)
            groundRb.AddForceAtPosition(-force, groundPoint, ForceMode.Force);
    }

    private void ApplyTireImpulse(
        Vector3 impulse,
        Vector3 wheelPoint,
        Vector3 groundPoint,
        Rigidbody groundRb,
        Vector3 wheelAxisWorld)
    {
        bool useDecoupled = enableDecoupledTireForceApplication
                         && decoupleTireForceAndTorque;
        if (useDecoupled)
        {
            Rigidbody.AddForce(impulse, ForceMode.Impulse);
            if (wheelAxisWorld.sqrMagnitude > 1e-10f)
            {
                wheelAxisWorld.Normalize();
                Vector3 angularImpulse = Vector3.Cross(
                    wheelPoint - Rigidbody.worldCenterOfMass,
                    impulse);
                float spinImpulse = Vector3.Dot(angularImpulse, wheelAxisWorld);
                if (Mathf.Abs(spinImpulse) > 1e-8f)
                    Rigidbody.AddTorque(
                        wheelAxisWorld * spinImpulse,
                        ForceMode.Impulse);
            }
        }
        else
        {
            Rigidbody.AddForceAtPosition(impulse, wheelPoint, ForceMode.Impulse);
        }

        if (groundRb != null)
            groundRb.AddForceAtPosition(-impulse, groundPoint, ForceMode.Impulse);
    }

    private Vector3 LimitVectorMagnitude(Vector3 v, float maxMagnitude)
    {
        if (maxMagnitude <= 0f) return Vector3.zero;
        float sqr = v.sqrMagnitude;
        float maxSqr = maxMagnitude * maxMagnitude;
        if (sqr <= maxSqr || sqr <= 1e-12f) return v;
        return v * (maxMagnitude / Mathf.Sqrt(sqr));
    }

    private float GetCombinedFrictionScale(Vector3 force, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        if (force.sqrMagnitude <= 1e-12f) return 1f;
        float fx = Vector3.Dot(force, forward);
        float fy = Vector3.Dot(force, side);
        float usage = 0f;

        if (maxLong > 1e-6f) usage += fx * fx / (maxLong * maxLong);
        else if (Mathf.Abs(fx) > 1e-6f) return 0f;
        if (maxSide > 1e-6f) usage += fy * fy / (maxSide * maxSide);
        else if (Mathf.Abs(fy) > 1e-6f) return 0f;

        return usage <= 1f ? 1f : 1f / Mathf.Sqrt(usage);
    }

    private Vector3 ClampCombinedTireForce(Vector3 force, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        float fx = Vector3.Dot(force, forward);
        float fy = Vector3.Dot(force, side);
        Vector3 tangentForce = forward * fx + side * fy;
        return tangentForce * GetCombinedFrictionScale(tangentForce, forward, side, maxLong, maxSide);
    }

    private Vector3 BuildCombinedKineticFriction(Vector3 vSlip, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        float vx = Vector3.Dot(vSlip, forward);
        float vy = Vector3.Dot(vSlip, side);
        float weightedLong = Mathf.Max(0f, maxLong) * vx;
        float weightedSide = Mathf.Max(0f, maxSide) * vy;
        float denominator = Mathf.Sqrt(
            weightedLong * weightedLong + weightedSide * weightedSide);
        if (denominator <= 1e-8f) return Vector3.zero;

        // Maximum-dissipation direction on an anisotropic friction ellipse:
        // F = -(a^2*vx, b^2*vy) / sqrt((a*vx)^2 + (b*vy)^2).
        // Unlike per-axis Sign(), tiny lateral numerical noise cannot request
        // full lateral grip and steal the longitudinal force budget.
        float fx = -Mathf.Max(0f, maxLong) * weightedLong / denominator;
        float fy = -Mathf.Max(0f, maxSide) * weightedSide / denominator;
        return forward * fx + side * fy;
    }

    private Vector3 BuildKineticTireForce(
        Vector3 slipVelocity,
        Vector3 forward,
        Vector3 side,
        float normalLoad)
    {
        if (slipVelocity.sqrMagnitude <= 1e-10f || normalLoad <= 1e-6f)
            return Vector3.zero;
        if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
        {
            return BuildCombinedKineticFriction(
                slipVelocity, forward, side,
                MuKineticEff() * Mathf.Max(0f, longitudinalGripScale) * normalLoad,
                MuKineticEff() * Mathf.Max(0f, lateralGripScale) * normalLoad);
        }
        return -slipVelocity.normalized * (MuKineticEff() * normalLoad);
    }

    private Vector3 BuildLowSpeedCreepForce(Vector3 vSlip, Vector3 forward, Vector3 side, float normalLoad)
    {
        if (!enableModernLowSpeedTire || normalLoad <= 1e-6f) return Vector3.zero;
        float creepV = Mathf.Max(1e-4f, lowSpeedCreepSpeed);

        if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
        {
            float maxLong = MuStaticEff() * Mathf.Max(0f, longitudinalGripScale) * normalLoad;
            float maxSide = MuStaticEff() * Mathf.Max(0f, lateralGripScale) * normalLoad;
            Vector3 demand = Vector3.zero;
            float vx = Vector3.Dot(vSlip, forward);
            if (Mathf.Abs(vx) > 1e-6f && maxLong > 0f)
                demand += -Mathf.Clamp(vx / creepV, -1f, 1f) * maxLong * forward;
            float vy = Vector3.Dot(vSlip, side);
            if (Mathf.Abs(vy) > 1e-6f && maxSide > 0f)
                demand += -Mathf.Clamp(vy / creepV, -1f, 1f) * maxSide * side;
            return ClampCombinedTireForce(demand, forward, side, maxLong, maxSide);
        }

        float speed = vSlip.magnitude;
        if (speed <= 1e-6f) return Vector3.zero;
        float demandMagnitude = MuStaticEff() * normalLoad * Mathf.Clamp01(speed / creepV);
        return -vSlip / speed * demandMagnitude;
    }

    // A5 anti-wobble: at high spin the joint solver cannot hold axle alignment
    // and precession grows ("wobble"). Damp the perpendicular-to-axle component
    // of the wheel-vs-joint-parent angular velocity. RELATIVE angular velocity
    // is essential: vehicle yaw/steering/suspension articulation must not be
    // resisted. Engagement ramps in with |omegaRel| so the damper is inert at
    // normal speeds. One-sided on purpose: this is an energy sink, and reacting
    // the impulse on the parent would feed the wobble back into the chassis.
    private const float WobbleDampingFraction = 0.25f;
    private const float WobbleDampingStartSpeed = 30f;
    private const float WobbleDampingFullSpeed = 60f;

    private void ApplyAxleWobbleDamping(Vector3 wheelAxisWorld)
    {
        if (!HasRigidbody) return;
        if (wheelAxisWorld.sqrMagnitude < 1e-10f) return;
        wheelAxisWorld.Normalize();

        Vector3 omegaRel = Rigidbody.angularVelocity;
        Rigidbody parentBody = GetJointParentBody();
        if (parentBody != null) omegaRel -= parentBody.angularVelocity;

        float engage = Mathf.Clamp01(
            (omegaRel.magnitude - WobbleDampingStartSpeed)
            / (WobbleDampingFullSpeed - WobbleDampingStartSpeed));
        if (engage <= 1e-4f) return;

        Vector3 omegaPerp = omegaRel
            - Vector3.Dot(omegaRel, wheelAxisWorld) * wheelAxisWorld;
        float perpSpeed = omegaPerp.magnitude;
        if (perpSpeed <= 1e-4f) return;

        float inertia = GetInertiaAroundWorldAxis(omegaPerp / perpSpeed);
        if (inertia <= 1e-6f) return;

        Rigidbody.AddTorque(
            omegaPerp * (-inertia * WobbleDampingFraction * engage),
            ForceMode.Impulse);
    }

    private void ApplyAxleSpinStabilization(float normalLoad, float radius, Vector3 wheelAxisWorld)
    {
        if (!enableModernLowSpeedTire || !HasRigidbody) return;
        if (wheelAxisWorld.sqrMagnitude < 1e-10f) return;
        wheelAxisWorld.Normalize();

        float omega = Vector3.Dot(Rigidbody.angularVelocity, wheelAxisWorld);
        if (Mathf.Abs(omega) <= 1e-5f) return;
        bool hasLoad = normalLoad > 1e-6f && radius > 1e-6f;
        // Loaded-wheel absolute omega damping fights pure rolling and destroys
        // coasting. Contact-patch friction now handles loaded stabilization;
        // this helper is only a free-spin bearing/air damping fallback.
        if (hasLoad) return;
        float dampingK = axleAirDampingK;
        if (dampingK <= 0f) return;

        float tau = -omega * dampingK;
        float inertia = GetInertiaAroundWorldAxis(wheelAxisWorld);
        float dt = Mathf.Max(1e-5f, Time.fixedDeltaTime);
        if (inertia > 1e-6f)
        {
            float stopTorque = Mathf.Abs(omega) * inertia / dt;
            if (stopTorque > 1e-6f) tau = Mathf.Clamp(tau, -stopTorque, stopTorque);
        }

        if (Mathf.Abs(tau) > 1e-6f)
            Rigidbody.AddTorque(wheelAxisWorld * tau, ForceMode.Force);
    }

    private float GetInertiaAroundWorldAxis(Vector3 axisWorld)
    {
        if (!HasRigidbody || axisWorld.sqrMagnitude < 1e-10f) return 0f;
        axisWorld.Normalize();
        Quaternion principalToWorld = Rigidbody.rotation * Rigidbody.inertiaTensorRotation;
        Vector3 axisPrincipal = Quaternion.Inverse(principalToWorld) * axisWorld;
        if (axisPrincipal.sqrMagnitude > 1e-10f) axisPrincipal.Normalize();
        Vector3 inertia = Rigidbody.inertiaTensor;
        return Mathf.Max(0f, inertia.x) * axisPrincipal.x * axisPrincipal.x
             + Mathf.Max(0f, inertia.y) * axisPrincipal.y * axisPrincipal.y
             + Mathf.Max(0f, inertia.z) * axisPrincipal.z * axisPrincipal.z;
    }
}
