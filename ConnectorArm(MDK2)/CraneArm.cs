using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public class CraneArm
        {
            private Rotor _joint0;
            private Rotor _joint1;
            private Vector3 _seg1Vector;
            private Rotor _joint2;
            private Vector3 _seg2Vector;
            private Rotor _eePitchJoint;
            private Rotor _eeYawJoint;
            private Rotor _eeRollJoint;

            private float _sensitivity;
            private float _speed;
            private bool _cylindricalMode;
            private bool _OOB = false;

            private Vector3 _targetCoord;
            private Matrix _targetEEO_base;

            public bool EEControlled { get; private set; } = false;
            public bool ArmControlled { get; private set; } = true;

            public CraneArm(float sensitivity, float speed, bool cylindricalMode, float seg1Length, float seg2Length)
            {
                _joint0 = new Rotor("Joint0");
                _joint1 = new Rotor("Joint1");
                _joint2 = new Rotor("Joint2");
                _eePitchJoint = new Rotor("EE Pitch Joint");
                _eeYawJoint = new Rotor("EE Yaw Joint");
                _eeRollJoint = new Rotor("EE Roll Joint");

                _sensitivity = sensitivity;
                _speed = speed;
                _cylindricalMode = cylindricalMode;

                _seg1Vector = new Vector3(0, 0, -seg1Length);
                _seg2Vector = new Vector3(0, 0, -seg2Length);

                Init();
            }

            private void Init()
            {
                Matrix H0 = Matrix.CreateRotationY(_joint0.CurrentAngle);
                Matrix H1 = Matrix.CreateRotationX(_joint1.CurrentAngle);
                H1.Translation = new Vector3(0, 2.5f, 0);
                Matrix H2 = Matrix.CreateRotationX(_joint2.CurrentAngle);
                H2.Translation = _seg1Vector;
                Matrix H3 = Matrix.Identity;
                H3.Translation = _seg2Vector;

                Matrix HT = H3 * H2 * H1 * H0;
                Vector3 currentCoord = HT.Translation;
                _targetCoord = currentCoord;

                if (!EEControlled)
                {
                    LockEEO(HT);
                }
            }

            public void Control(UserInput input)
            {
                Matrix H0 = Matrix.CreateRotationY(_joint0.CurrentAngle);
                Matrix H1 = Matrix.CreateRotationX(_joint1.CurrentAngle);
                H1.Translation = new Vector3(0, 2.5f, 0);
                Matrix H2 = Matrix.CreateRotationX(_joint2.CurrentAngle);
                H2.Translation = _seg1Vector;
                Matrix H3 = Matrix.Identity;
                H3.Translation = _seg2Vector;

                Matrix HT = H3 * H2 * H1 * H0;
                Vector3 currentCoord = HT.Translation;

                float minTargetDistance = (_seg1Vector - _seg2Vector).Length() + 1;
                float maxTargetDistance = (_seg1Vector + _seg2Vector).Length();

                if (EEControlled == true)
                {
                    _eeYawJoint.Velocity = -0.01f * _sensitivity * input.MouseInput.Y;
                    _eePitchJoint.Velocity = -0.01f * _sensitivity * input.MouseInput.X;
                    _eeRollJoint.Velocity = input.QPress ? -0.05f * _sensitivity : input.EPress ? 0.05f * _sensitivity : 0f;
                }

                _targetCoord.Z += input.WPress ? -0.05f * _sensitivity : input.SPress ? 0.05f * _sensitivity : 0f;
                _targetCoord.Y += input.CPress ? -0.05f * _sensitivity : input.SpacePress ? 0.05f * _sensitivity : 0f;
                _targetCoord.X += input.APress ? -0.05f * _sensitivity : input.DPress ? 0.05f * _sensitivity : 0f;

                float targetDistance = _targetCoord.Length();

                float X = ((targetDistance * targetDistance) - Vector3.Dot(_seg2Vector, _seg2Vector) + Vector3.Dot(_seg1Vector, _seg1Vector)) / (2 * targetDistance);
                float Y = (float)Math.Sqrt(Vector3.Dot(_seg1Vector, _seg1Vector) - (X * X));

                float joint0Target = (float)Math.Atan2(-_targetCoord.X, -_targetCoord.Z);
                float joint1Target = (float)Math.Atan2(Y, X) + (float)Math.Asin(MathHelper.Clamp(_targetCoord.Y / targetDistance, -1f, 1f));
                float joint2Target = (float)Math.Atan2(0 - Y, targetDistance - X) - (float)Math.Atan2(Y, X);

                if (targetDistance > maxTargetDistance || targetDistance < minTargetDistance)
                {
                    _OOB = true;
                    _targetCoord = currentCoord;
                    return;
                }
                if (joint0Target < _joint0.MinAngle || joint0Target > _joint0.MaxAngle)
                {
                    _OOB = true;
                    _targetCoord = currentCoord;
                    return;
                }
                if (joint1Target < _joint1.MinAngle || joint1Target > _joint1.MaxAngle)
                {
                    _OOB = true;
                    _targetCoord = currentCoord;
                    return;
                }
                if (joint2Target < _joint2.MinAngle || joint2Target > _joint2.MaxAngle)
                {
                    _OOB = true;
                    _targetCoord = currentCoord;
                    return;
                }

                if (!EEControlled)
                {
                    Matrix H0T = Matrix.CreateRotationY(joint0Target);
                    Matrix H1T = Matrix.CreateRotationX(joint1Target);
                    Matrix H2T = Matrix.CreateRotationX(joint2Target);
                    H2T.Translation = _seg1Vector;
                    Matrix H3T = Matrix.Identity;
                    H3T.Translation = _seg2Vector;

                    Matrix HTT = H3 * H2 * H1 * H0;


                    Matrix targetEEO_ee = _targetEEO_base * Matrix.Transpose(HTT.GetOrientation());
                    float targetPitch = (float)Math.Asin(MathHelper.Clamp(-targetEEO_ee.M32, -1f, 1f));
                    float targetYaw;
                    float targetRoll;

                    float epsilon = 0.001f;
                    if (targetEEO_ee.M32 < -1 + epsilon)
                    {
                        targetRoll = 0;
                        targetYaw = (float)Math.Atan2(targetEEO_ee.M21, targetEEO_ee.M11);
                    }
                    else if (targetEEO_ee.M32 > 1 - epsilon)
                    {
                        targetRoll = 0;
                        targetYaw = (float)Math.Atan2(-targetEEO_ee.M21, targetEEO_ee.M11);
                    }
                    else
                    {
                        targetRoll = (float)Math.Atan2(targetEEO_ee.M12, targetEEO_ee.M22);
                        targetYaw = (float)Math.Atan2(targetEEO_ee.M31, targetEEO_ee.M33);
                    }

                    float yawError = targetYaw - _eeYawJoint.CurrentAngle;
                    yawError = MiscUtilities.LoopInRange(yawError, -(float)Math.PI, (float)Math.PI);

                    float pitchError = targetPitch - _eePitchJoint.CurrentAngle;
                    pitchError = MiscUtilities.LoopInRange(pitchError, -(float)Math.PI, (float)Math.PI);

                    float rollError = targetRoll - _eeRollJoint.CurrentAngle;
                    rollError = MiscUtilities.LoopInRange(rollError, -(float)Math.PI, (float)Math.PI);

                    if (!float.IsNaN(yawError) && !float.IsNaN(pitchError) && !float.IsNaN(rollError))
                    {
                        _eeYawJoint.Velocity = yawError * _speed;
                        _eePitchJoint.Velocity = pitchError * _speed;
                        _eeRollJoint.Velocity = rollError * _speed;
                    }
                }
                

                float joint0Error = joint0Target - _joint0.CurrentAngle;
                joint0Error = MiscUtilities.LoopInRange(joint0Error, -(float)Math.PI, (float)Math.PI);

                float joint1Error = joint1Target - _joint1.CurrentAngle;
                joint1Error = MiscUtilities.LoopInRange(joint1Error, -(float)Math.PI, (float)Math.PI);

                float joint2Error = joint2Target - _joint2.CurrentAngle;
                joint2Error = MiscUtilities.LoopInRange(joint2Error, -(float)Math.PI, (float)Math.PI);

                

                if (!float.IsNaN(joint0Error) && !float.IsNaN(joint1Error) && !float.IsNaN(joint2Error))
                {
                    _joint0.Velocity = joint0Error * _speed;
                    _joint1.Velocity = joint1Error * _speed;
                    _joint2.Velocity = joint2Error * _speed;
                }
            }

            public void LockEEO(Matrix baseEEO)
            {
                Quaternion q0 = Quaternion.CreateFromAxisAngle(baseEEO.Up, _eeYawJoint.CurrentAngle);
                Quaternion q1 = Quaternion.CreateFromAxisAngle(baseEEO.Right, _eePitchJoint.CurrentAngle);
                Quaternion q2 = Quaternion.CreateFromAxisAngle(baseEEO.Backward, _eeRollJoint.CurrentAngle);
                Quaternion qT = q0 * q1 * q2;
                Matrix currentEEO_base = Matrix.Transform(baseEEO.GetOrientation(), qT);
                _targetEEO_base = currentEEO_base;
            }
        }
    }
}
