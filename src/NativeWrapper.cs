// For examples, see:
// https://thegraybook.vvvv.org/reference/extending/writing-nodes.html#examples

using System.Runtime.InteropServices;
using System;
using System.Linq;
using System.Collections.Generic;
using VL.Lib.Collections;
using Vector3 = Stride.Core.Mathematics.Vector3;
using Vector4 = Stride.Core.Mathematics.Vector4;
using Quaternion = Stride.Core.Mathematics.Quaternion;
using Mat = Stride.Core.Mathematics.Matrix;

namespace Dope;

public static class DopeNativeWrapper
{
    [DllImport("BBW4vvvv.dll")]
    private static extern IntPtr ComputeBBW(double[] Vertices, int[] TI, double[] ControlV, int[] PointI, int[] BoneEdgeI, int[] CageEdgeI, int[] binSizes, bool isTetra);//, IntPtr PseudoEdgeI);

    [DllImport("BBW4vvvv.dll")]
    private static extern IntPtr forward_kinematics(IntPtr dQ, IntPtr dT, IntPtr Cext, IntPtr BEext, IntPtr binSizes);

    [DllImport("BBW4vvvv.dll")]
    private static extern int ReleaseMemory(IntPtr ptr);

    public static IEnumerable<Mat> ForwardKinematics(IEnumerable<Vector3> FCV, IEnumerable<int> FBEi, IEnumerable<Vector4> FRot,
                            IEnumerable<Vector3> FTrans)
    {
        // materialize inputs to avoid multiple enumeration
        var fcvarr = FCV?.ToArray() ?? Array.Empty<Vector3>();
        var fbearr = FBEi?.ToArray() ?? Array.Empty<int>();
        var frots = FRot?.ToArray() ?? Array.Empty<Vector4>();
        var ftrans = FTrans?.ToArray() ?? Array.Empty<Vector3>();

        int cPoints = fcvarr.Length;
        int cPointsXYZ = cPoints * 3;
        int boneEdgesXY = fbearr.Length;
        int boneEdges = boneEdgesXY / 2;

        var help = new Helpers();

        double[] C = new double[cPointsXYZ];
        C = help.Vector3DSequenceToArray(C, fcvarr);

        int[] BE = fbearr;

        double[] Q = new double[boneEdges * 4];
        for (int i = 0; i < boneEdges && i < frots.Length; i++)
        {
            Q[i * 4] = frots[i].W;
            Q[i * 4 + 1] = frots[i].X;
            Q[i * 4 + 2] = frots[i].Y;
            Q[i * 4 + 3] = frots[i].Z;
        }

        double[] T = new double[boneEdges * 3];
        for (int i = 0; i < boneEdges && i < ftrans.Length; i++)
        {
            T[i * 3] = ftrans[i].X;
            T[i * 3 + 1] = ftrans[i].Y;
            T[i * 3 + 2] = ftrans[i].Z;
        }

        int[] binSizes = new int[] { cPoints, boneEdges };

        // Allocate unmanaged memory for all arrays
        IntPtr Qptr = Marshal.AllocHGlobal(Q.Length * sizeof(double));
        IntPtr Tptr = Marshal.AllocHGlobal(T.Length * sizeof(double));
        IntPtr Cptr = Marshal.AllocHGlobal(C.Length * sizeof(double));
        IntPtr BEptr = Marshal.AllocHGlobal(BE.Length * sizeof(int));
        IntPtr binSizesPtr = Marshal.AllocHGlobal(binSizes.Length * sizeof(int));

        try
        {
            // Copy managed arrays to unmanaged memory
            Marshal.Copy(Q, 0, Qptr, Q.Length);
            Marshal.Copy(T, 0, Tptr, T.Length);
            Marshal.Copy(C, 0, Cptr, C.Length);
            Marshal.Copy(BE, 0, BEptr, BE.Length);
            Marshal.Copy(binSizes, 0, binSizesPtr, binSizes.Length);

            // Call native function with unmanaged pointers
            IntPtr tMatrix = forward_kinematics(Qptr, Tptr, Cptr, BEptr, binSizesPtr);

            int tMatrixSize = 12 * boneEdges;
            double[] tMatrixArr = new double[tMatrixSize];
            Marshal.Copy(tMatrix, tMatrixArr, 0, tMatrixSize);

            var result = new List<Mat>(boneEdges);
            for (int i = 0; i < boneEdges; i++)
            {
                int b = i * 12;
                Mat mTemp = new Mat();
                mTemp.Row1 = new Vector4((float)tMatrixArr[b + 0], (float)tMatrixArr[b + 1], (float)tMatrixArr[b + 2], 0.0f);
                mTemp.Row2 = new Vector4((float)tMatrixArr[b + 3], (float)tMatrixArr[b + 4], (float)tMatrixArr[b + 5], 0.0f);
                mTemp.Row3 = new Vector4((float)tMatrixArr[b + 6], (float)tMatrixArr[b + 7], (float)tMatrixArr[b + 8], 0.0f);
                mTemp.Row4 = new Vector4((float)tMatrixArr[b + 9], (float)tMatrixArr[b + 10], (float)tMatrixArr[b + 11], 1.0f);
                result.Add(mTemp);
            }

            ReleaseMemory(tMatrix);

            return result;
        }
        finally
        {
            // Free all allocated unmanaged memory
            Marshal.FreeHGlobal(Qptr);
            Marshal.FreeHGlobal(Tptr);
            Marshal.FreeHGlobal(Cptr);
            Marshal.FreeHGlobal(BEptr);
            Marshal.FreeHGlobal(binSizesPtr);
        }
    }

    /// <summary>
    /// Solve inverse kinematics for bone chains using a FABRIK-like solver.
    /// Assumptions:
    /// - Bone segments are pairs of indices into the control point array (BE: [a0,b0,a1,b1,...]).
    /// - The bone edges form one or more chains (graph degree &lt;= 2). Branching is not supported.
    /// - targets: one translation (target position) per chain end-point. Order of targets must match the order of discovered chain end-points.
    /// - If allowScaling is true, segment lengths may be scaled uniformly to attempt to reach the target.
    /// Returns a matrix per bone segment in the same order as the input bone edge indices. Each matrix is 4x4 stored in a Stride Matrix (rows filled similarly to ForwardKinematics).
    /// </summary>
   /// <summary>
/// Solve inverse kinematics for bone chains using a FABRIK-like solver with proper transform hierarchy.
/// Returns connected bone matrices that form proper chains.
/// </summary>
/// <summary>
/// Solve inverse kinematics for bone chains using a FABRIK-like solver with proper transform hierarchy.
/// Returns connected bone matrices that form proper chains.
/// </summary>
public static IEnumerable<Mat> InverseKinematics(
    IEnumerable<Vector3> controlPoints,
    IEnumerable<int> boneEdgeIndices,
    IEnumerable<Vector3> targets,
    bool allowScaling = false,
    bool fixStartpoint = true,
    int maxIterations = 20,
    double tolerance = 1e-4,
    bool targetsAreOffsets = false,
    string solver = "FABRIK",
    IEnumerable<int> constrainedJointIndices = null,
    IEnumerable<Vector3> constraintAxes = null,
    IEnumerable<double> constraintMinAngles = null,
    IEnumerable<double> constraintMaxAngles = null)
{
    // Ported and extended: supports FABRIK (default) and a simplified CCDIK with optional per-joint angular limits.
    var pts = controlPoints?.ToArray() ?? Array.Empty<Vector3>();
    var be = boneEdgeIndices?.ToArray() ?? Array.Empty<int>();
    var tgt = targets?.ToArray() ?? Array.Empty<Vector3>();

    if (be.Length % 2 != 0) throw new ArgumentException("boneEdgeIndices must contain pairs of indices.", nameof(boneEdgeIndices));

    int boneCount = be.Length / 2;
    int pointCount = pts.Length;

    // Build adjacency and bone->index map
    var adjacency = new List<int>[pointCount];
    for (int i = 0; i < pointCount; i++) adjacency[i] = new List<int>();
    var boneList = new (int a, int b)[boneCount];
    var boneIndexMap = new Dictionary<(int, int), int>();

    for (int i = 0; i < boneCount; i++)
    {
        int a = be[i * 2];
        int b = be[i * 2 + 1];
        boneList[i] = (a, b);
        adjacency[a].Add(b);
        adjacency[b].Add(a);
        boneIndexMap[(a, b)] = i;
        boneIndexMap[(b, a)] = i;
    }

    // Find chain endpoints
    var endpoints = new List<int>();
    for (int i = 0; i < pointCount; i++)
        if (adjacency[i].Count == 1) endpoints.Add(i);

    if (endpoints.Count == 0)
        throw new NotSupportedException("Closed-loop chains are not supported by this IK implementation.");

    // Build chains
    var chains = new List<List<int>>();
    var visitedForChains = new bool[pointCount];

    foreach (var start in endpoints)
    {
        if (visitedForChains[start]) continue;
        var chain = new List<int>();
        int prev = -1;
        int cur = start;

        while (true)
        {
            chain.Add(cur);
            visitedForChains[cur] = true;
            var neighbors = adjacency[cur];
            int next = -1;

            foreach (var n in neighbors)
            {
                if (n == prev) continue;
                next = n;
                break;
            }

            if (next == -1) break;
            prev = cur;
            cur = next;
            if (visitedForChains[cur]) break;
        }

        chains.Add(chain);
    }

    // Map targets to endpoints
    var endpointToTarget = new Dictionary<int, Vector3>();

    if (tgt.Length >= chains.Count && chains.Count > 0)
    {
        for (int i = 0; i < chains.Count; i++)
        {
            var chain = chains[i];
            int actuator = chain[chain.Count - 1];
            var provided = tgt[i];
            endpointToTarget[actuator] = targetsAreOffsets ? (pts[actuator] + provided) : provided;
        }
    }
    else if (tgt.Length == endpoints.Count && endpoints.Count > 0)
    {
        for (int i = 0; i < endpoints.Count; i++)
        {
            int ep = endpoints[i];
            var provided = tgt[i];
            endpointToTarget[ep] = targetsAreOffsets ? (pts[ep] + provided) : provided;
        }
    }
    else if (tgt.Length > 0 && tgt.Length < chains.Count)
    {
        int m = tgt.Length;
        for (int i = 0; i < m; i++)
        {
            var chain = chains[i];
            int actuator = chain[chain.Count - 1];
            var provided = tgt[i];
            endpointToTarget[actuator] = targetsAreOffsets ? (pts[actuator] + provided) : provided;
        }
    }

    // Constraint maps (optional)
    var constrainedMap = new Dictionary<int, (Vector3 axis, double minA, double maxA)>();
    if (constrainedJointIndices != null && constraintAxes != null && constraintMinAngles != null && constraintMaxAngles != null)
    {
        var idxs = constrainedJointIndices.ToArray();
        var axes = constraintAxes.ToArray();
        var mins = constraintMinAngles.ToArray();
        var maxs = constraintMaxAngles.ToArray();
        int m = Math.Min(idxs.Length, Math.Min(axes.Length, Math.Min(mins.Length, maxs.Length)));
        for (int i = 0; i < m; i++) constrainedMap[idxs[i]] = (axes[i], mins[i], maxs[i]);
    }

    // Solved positions start as the control points
    var solvedPositions = pts.Select(p => new Vector3(p.X, p.Y, p.Z)).ToArray();

    if (string.Equals(solver, "CCDIK", StringComparison.OrdinalIgnoreCase))
    {
        // Simple CCD implementation operating on positions. Applies optional per-joint angular clamping (in degrees).
        foreach (var rawChain in chains)
        {
            if (rawChain.Count < 2) continue;

            // Make a local copy so we can reverse orientation when needed
            var chain = new List<int>(rawChain);

            // Determine actuator (the joint that has a target). Prefer the last element, but if the first element
            // has a target, orient the chain so the actuator is the last element. This ensures consistent root->effector ordering.
            bool firstHasTarget = endpointToTarget.ContainsKey(chain[0]);
            bool lastHasTarget = endpointToTarget.ContainsKey(chain[chain.Count - 1]);
            if (firstHasTarget && !lastHasTarget)
            {
                chain.Reverse();
            }

            int endIdx = chain[chain.Count - 1];
            Vector3 target = endpointToTarget.ContainsKey(endIdx) ? endpointToTarget[endIdx] : solvedPositions[endIdx];

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // early exit
                if (Vector3.Distance(solvedPositions[endIdx], target) <= tolerance) break;

                for (int j = chain.Count - 2; j >= 0; j--)
                {
                    int jointIdx = chain[j];
                    Vector3 jointPos = solvedPositions[jointIdx];

                    Vector3 effPos = solvedPositions[endIdx];
                    Vector3 toEff = effPos - jointPos;
                    Vector3 toTarget = target - jointPos;

                    float lenToEff = toEff.Length();
                    float lenToTarget = toTarget.Length();
                    if (lenToEff < 1e-9f || lenToTarget < 1e-9f) continue;

                    var v1 = Vector3.Normalize(toEff);
                    var v2 = Vector3.Normalize(toTarget);

                    float dot = Math.Clamp(Vector3.Dot(v1, v2), -1f, 1f);
                    float angle = (float)Math.Acos(dot); // radians

                    if (angle < 1e-6f) continue;

                    var axis = Vector3.Cross(v1, v2);
                    if (axis.LengthSquared() < 1e-18f) continue;
                    axis = Vector3.Normalize(axis);

                    // Apply per-joint constraint if present (clamp angle in degrees)
                    if (constrainedMap.TryGetValue(jointIdx, out var constraint))
                    {
                        double minDeg = constraint.minA;
                        double maxDeg = constraint.maxA;
                        double angDeg = angle * (180.0 / Math.PI);
                        double clamped = Math.Max(minDeg, Math.Min(maxDeg, angDeg));
                        angle = (float)(clamped * (Math.PI / 180.0));
                    }

                    Quaternion q = new Quaternion();
                    Quaternion.RotationAxis(axis, angle, out q);
                    // Rotate all downstream points (j+1 .. end)
                    for (int k = j + 1; k < chain.Count; k++)
                    {
                        int pi = chain[k];
                        Vector3 rel = solvedPositions[pi] - jointPos;
                        Vector3 rot = Vector3.Transform(rel, q);
                        solvedPositions[pi] = jointPos + rot;
                    }
                }
            }
        }
    }
    else // default FABRIK
    {
        // Iterate the prebuilt chains, ensure each chain is oriented root->actuator (actuator = last element)
        foreach (var rawChain in chains)
        {
            if (rawChain.Count < 2) continue;

            var chain = new List<int>(rawChain);
            bool firstHasTarget = endpointToTarget.ContainsKey(chain[0]);
            bool lastHasTarget = endpointToTarget.ContainsKey(chain[chain.Count - 1]);
            if (firstHasTarget && !lastHasTarget)
            {
                chain.Reverse();
            }

            int end = chain[chain.Count - 1];
            Vector3 targetA = endpointToTarget.ContainsKey(chain[0]) ? endpointToTarget[chain[0]] : solvedPositions[chain[0]];
            Vector3 targetB = endpointToTarget.ContainsKey(end) ? endpointToTarget[end] : solvedPositions[end];

            int chainLen = chain.Count;
            var positions = new Vector3[chainLen];
            for (int i = 0; i < chainLen; i++) positions[i] = solvedPositions[chain[i]];

            double[] lengths = new double[Math.Max(0, chainLen - 1)];
            double totalLength = 0;
            for (int i = 0; i < chainLen - 1; i++)
            {
                lengths[i] = Vector3.Distance(positions[i], positions[i + 1]);
                totalLength += lengths[i];
            }

            double targetDist = Vector3.Distance(targetA, targetB);
            bool reachable = targetDist <= totalLength || allowScaling;

            if (!reachable && !allowScaling)
            {
                if (fixStartpoint || endpointToTarget.ContainsKey(chain[0]))
                {
                    positions[0] = targetA;
                    var direction = targetB - targetA;
                    double distAB = Vector3.Distance(targetA, targetB);
                    if (distAB < 1e-12) distAB = 1e-12;

                    for (int i = 0; i < chainLen - 1; i++)
                    {
                        double r = lengths[i] / distAB;
                        positions[i + 1] = positions[i] + (float)r * direction;
                    }
                }
                else
                {
                    positions[chainLen - 1] = targetB;
                    var dir = Vector3.Zero;
                    for (int i = chainLen - 2; i >= 0; i--)
                    {
                        dir = Vector3.Normalize(positions[i] - positions[i + 1]);
                        if (dir.LengthSquared() < 1e-18f) dir = new Vector3(0, 0, 1);
                        positions[i] = positions[i + 1] + dir * (float)lengths[i];
                    }
                }
            }
            else
            {
                if (allowScaling && targetDist > totalLength && totalLength > 1e-12)
                {
                    double scale = targetDist / totalLength;
                    for (int i = 0; i < lengths.Length; i++) lengths[i] *= scale;
                    totalLength = targetDist;
                }

                for (int iter = 0; iter < maxIterations; iter++)
                {
                    // Forward reaching
                    positions[chainLen - 1] = targetB;
                    for (int i = chainLen - 2; i >= 0; i--)
                    {
                        double r = Vector3.Distance(positions[i + 1], positions[i]);
                        if (r < 1e-12) continue;
                        double lambda = lengths[i] / r;
                        positions[i] = Vector3.Lerp(positions[i + 1], positions[i], (float)lambda);
                    }

                    // Backward reaching
                    if (fixStartpoint || endpointToTarget.ContainsKey(chain[0]))
                    {
                        positions[0] = targetA;
                    }

                    for (int i = 0; i < chainLen - 1; i++)
                    {
                        double r = Vector3.Distance(positions[i + 1], positions[i]);
                        if (r < 1e-12) continue;
                        double lambda = lengths[i] / r;
                        positions[i + 1] = Vector3.Lerp(positions[i], positions[i + 1], (float)lambda);
                    }

                    if (Vector3.Distance(positions[chainLen - 1], targetB) <= tolerance) break;
                }
            }

            for (int i = 0; i < chainLen; i++)
                solvedPositions[chain[i]] = positions[i];
        }
    }

    // ===== Build bone matrices =====
    var results = new Mat[boneCount];

    // Initialize all to identity
    for (int i = 0; i < boneCount; i++)
    {
        results[i] = Mat.Identity;
    }

    // Process each chain to build hierarchical transforms
    foreach (var chain in chains)
    {
        if (chain.Count < 2) continue;

        for (int i = 0; i < chain.Count - 1; i++)
        {
            int pointA = chain[i];
            int pointB = chain[i + 1];

            // Find bone index
            if (!boneIndexMap.TryGetValue((pointA, pointB), out int boneIdx))
                continue;

            // Get the canonical bone direction (as defined in boneList)
            var (canonA, canonB) = boneList[boneIdx];

            // Build rest matrix (from original control points)
            Vector3 restPosA = pts[canonA];
            Vector3 restPosB = pts[canonB];
            Vector3 restDir = restPosB - restPosA;
            Mat restMatrix = BuildBoneMatrix(restDir, restPosA);

            // Build solved matrix (from result)
            Vector3 solvedPosA = solvedPositions[canonA];
            Vector3 solvedPosB = solvedPositions[canonB];
            Vector3 solvedDir = solvedPosB - solvedPosA;
            Mat solvedMatrix = BuildBoneMatrix(solvedDir, solvedPosA);

            // If the caller requested offsets (relative targets) they probably expect the returned
            // transform to be relative to the rest (bind) pose: returned = inv(rest) * solved.
            // Otherwise return absolute world transform.
            if (targetsAreOffsets)
            {
                // compute inv(rest) * solved
                // Stride Matrix doesn't have a direct static Invert that returns matrix, so use Invert on a copy
                Mat invRest = restMatrix;
                Mat.Invert(ref invRest, out invRest);
                results[boneIdx] = Mat.Multiply(invRest, solvedMatrix);
            }
            else
            {
                results[boneIdx] = solvedMatrix;
            }
        }
    }

    return results;
}

// Helper to build a bone matrix from direction and position
private static Mat BuildBoneMatrix(Vector3 direction, Vector3 position)
{
    float len = direction.Length();

    if (len < 1e-9f)
    {
        // Zero-length bone: identity rotation at position
        var mat = Mat.Identity;
        mat.M41 = position.X;
        mat.M42 = position.Y;
        mat.M43 = position.Z;
        return mat;
    }

    // Build orthonormal basis with bone pointing along the Y axis (Unity convention: bone.up = direction)
    var boneUp = Vector3.Normalize(direction);

    // Choose an arbitrary forward vector to build a basis; avoid near-parallel cases
    var arbitrary = new Vector3(0, 0, 1);
    if (Math.Abs(Vector3.Dot(boneUp, arbitrary)) > 0.999f)
        arbitrary = new Vector3(1, 0, 0);

    var right = Vector3.Normalize(Vector3.Cross(boneUp, arbitrary));
    var forward = Vector3.Cross(right, boneUp);

    // Build matrix (Stride uses row-major, so rows are basis vectors)
    // Row1 = right, Row2 = boneUp (bone pointing along Y), Row3 = forward
    return new Mat(
        right.X, right.Y, right.Z, 0,
        boneUp.X, boneUp.Y, boneUp.Z, 0,
        forward.X, forward.Y, forward.Z, 0,
        position.X, position.Y, position.Z, 1
    );
}

// Helper to build quaternion from direction (for future quaternion-based version)
private static Quaternion QuaternionFromDirection(Vector3 direction, Vector3 up)
{
    if (direction.LengthSquared() < 1e-9f)
        return Quaternion.Identity;

    direction = Vector3.Normalize(direction);
    
    if (Math.Abs(Vector3.Dot(direction, up)) > 0.999f)
        up = new Vector3(1, 0, 0);

    var right = Vector3.Normalize(Vector3.Cross(up, direction));
    var realUp = Vector3.Cross(direction, right);

    // Convert rotation matrix to quaternion (Stride-compatible)
    float trace = right.X + realUp.Y + direction.Z;
    Quaternion q;

    if (trace > 0.0f)
    {
        float s = (float)Math.Sqrt(trace + 1.0f);
        q.W = s * 0.5f;
        s = 0.5f / s;
        q.X = (realUp.Z - direction.Y) * s;
        q.Y = (direction.X - right.Z) * s;
        q.Z = (right.Y - realUp.X) * s;
    }
    else if (right.X >= realUp.Y && right.X >= direction.Z)
    {
        float s = (float)Math.Sqrt(1.0f + right.X - realUp.Y - direction.Z);
        float invS = 0.5f / s;
        q.X = 0.5f * s;
        q.Y = (right.Y + realUp.X) * invS;
        q.Z = (right.Z + direction.X) * invS;
        q.W = (realUp.Z - direction.Y) * invS;
    }
    else if (realUp.Y > direction.Z)
    {
        float s = (float)Math.Sqrt(1.0f + realUp.Y - right.X - direction.Z);
        float invS = 0.5f / s;
        q.X = (realUp.X + right.Y) * invS;
        q.Y = 0.5f * s;
        q.Z = (direction.Y + realUp.Z) * invS;
        q.W = (direction.X - right.Z) * invS;
    }
    else
    {
        float s = (float)Math.Sqrt(1.0f + direction.Z - right.X - realUp.Y);
        float invS = 0.5f / s;
        q.X = (direction.X + right.Z) * invS;
        q.Y = (direction.Y + realUp.Z) * invS;
        q.Z = 0.5f * s;
        q.W = (right.Y - realUp.X) * invS;
    }

    return q;
}


    public static IEnumerable<double> ComputeWeights(IEnumerable<Vector3> vertices, IEnumerable<int> indices, bool is3D, IEnumerable<Vector3> controls,
                                IEnumerable<int> pointHandleIndices, IEnumerable<int> boneEdgeIndices, IEnumerable<int> cageEdgeIndices/*, bool bind*/)
    {
        int dim;
        int entries = vertices.Count();
        int entriesXYz;
        int tritet0123 = indices.Count();
        int tritets;
        int cPoints = controls.Count();
        int cPointsXYz = cPoints * 3;

        //use bin size info!!
        //			int boneEdges = boneEdgesXY/2;
        SpreadBuilder<double> w = new SpreadBuilder<double>();


        int cageEdges = 0;
        int boneEdges = 0;
        int pointHandles = 0;
        if (cageEdgeIndices.All(c => c >= 0)) cageEdges = cageEdgeIndices.Count() / 2;
        if (boneEdgeIndices.All(c => c >= 0)) boneEdges = boneEdgeIndices.Count() / 2;
        if (pointHandleIndices.All(c => c >= 0)) pointHandles = pointHandleIndices.Count();



        if (is3D) dim = 3; else dim = 2;

        entriesXYz = entries * dim;
        cPointsXYz = cPoints * dim;
        tritets = tritet0123 / (dim + 1);

        int lbsSize = entries * (dim + 1) * (boneEdges + pointHandles);

        //FLbsArray.Count() = lbsSize;

        //       if (bind)
        //       {
        double[] V = new double[entriesXYz];
        double[] C = new double[cPointsXYz];

        var help = new Helpers();


        if (is3D)
        {
            V = help.Vector3DSequenceToArray(V, vertices);
            C = help.Vector3DSequenceToArray(C, controls);
        }
        else
        {
            V = help.Vector3DSequenceToArray2D(V, vertices);
            C = help.Vector3DSequenceToArray2D(C, controls);
        }


        int[] binSizes = new int[7];
        binSizes[0] = entries;
        binSizes[1] = tritets;
        binSizes[2] = cPoints;
        binSizes[3] = pointHandles;
        binSizes[4] = boneEdges;
        binSizes[5] = cageEdges;
        binSizes[6] = tritets;

        int[] T = new int[tritet0123];
        T = indices.ToArray();

        int[] BE = new int[boneEdgeIndices.Count()];
        BE = boneEdgeIndices.ToArray();

        int[] PH = new int[pointHandleIndices.Count()];
        PH = pointHandleIndices.ToArray();

        int[] CE = new int[cageEdgeIndices.Count()];
        CE = cageEdgeIndices.ToArray();



        try
        {

            IntPtr lbs = ComputeBBW(V, T, C, PH, BE, CE, binSizes, is3D);

            double[] lbsArr = new double[lbsSize];
            Marshal.Copy(lbs, lbsArr, 0, lbsSize);


            for (int i = 0; i < lbsSize; i++)
            {
                w.Add(lbsArr[i]);
            }

            ReleaseMemory(lbs);

        }
        //			catch (ex) {FLogger.Log(LogType.Debug, ex.Message);} 

        finally
        {

        }

        //       }
        return w;
    }

    private class Helpers
    {

        // Convert an IEnumerable<Vector3> into a flat double[] (x,y,z per element)
        public double[] Vector3DSequenceToArray(double[] V, IEnumerable<Vector3> VertexSequence)
        {
            var v = VertexSequence.ToArray();
            for (int i = 0; i < v.Length; i++)
            {
                V[i * 3] = v[i].X;
                V[i * 3 + 1] = v[i].Y;
                V[i * 3 + 2] = v[i].Z;
            }
            return V;
        }

        // Overload that accepts an already-materialized array to avoid extra allocations
        public double[] Vector3DSequenceToArray(double[] V, Vector3[] VertexArray)
        {
            for (int i = 0; i < VertexArray.Length; i++)
            {
                V[i * 3] = VertexArray[i].X;
                V[i * 3 + 1] = VertexArray[i].Y;
                V[i * 3 + 2] = VertexArray[i].Z;
            }
            return V;
        }

        // Convert an IEnumerable<Vector4> into a flat double[] (x,y,z,w per element)
        public double[] Vector4DSequenceToArray(double[] V, IEnumerable<Vector4> VertexSequence)
        {
            var v = VertexSequence.ToArray();
            for (int i = 0; i < v.Length; i++)
            {
                V[i * 4] = v[i].X;
                V[i * 4 + 1] = v[i].Y;
                V[i * 4 + 2] = v[i].Z;
                V[i * 4 + 3] = v[i].W;
            }
            return V;
        }

        // Overload for already-materialized Vector4[]
        public double[] Vector4DSequenceToArray(double[] V, Vector4[] VertexArray)
        {
            for (int i = 0; i < VertexArray.Length; i++)
            {
                V[i * 4] = VertexArray[i].X;
                V[i * 4 + 1] = VertexArray[i].Y;
                V[i * 4 + 2] = VertexArray[i].Z;
                V[i * 4 + 3] = VertexArray[i].W;
            }
            return V;
        }

        // Fill array in native quaternion order w,x,y,z
        public double[] Vector4DSequenceToArrayAsWXYZ(double[] V, IEnumerable<Vector4> VertexSequence)
        {
            var v = VertexSequence.ToArray();
            for (int i = 0; i < v.Length; i++)
            {
                V[i * 4] = v[i].W;
                V[i * 4 + 1] = v[i].X;
                V[i * 4 + 2] = v[i].Y;
                V[i * 4 + 3] = v[i].Z;
            }
            return V;
        }

        // Overload for already-materialized Vector4[]
        public double[] Vector4DSequenceToArrayAsWXYZ(double[] V, Vector4[] VertexArray)
        {
            for (int i = 0; i < VertexArray.Length; i++)
            {
                V[i * 4] = VertexArray[i].W;
                V[i * 4 + 1] = VertexArray[i].X;
                V[i * 4 + 2] = VertexArray[i].Y;
                V[i * 4 + 3] = VertexArray[i].Z;
            }
            return V;
        }

        public double[] Vector3DSequenceToArray2D(double[] V, IEnumerable<Vector3> VertexSequence)
        {
            Vector3[] v = VertexSequence.ToArray<Vector3>();

            for (int i = 0; i < v.Count(); i++)
            {
                V[i * 2] = v[i].X;
                V[i * 2 + 1] = v[i].Y;
            }
            return V;
        }

        // Overloads to copy from already materialized arrays
        public double[] Vector3DSequenceToArray2D(double[] V, Vector3[] VertexArray)
        {
            for (int i = 0; i < VertexArray.Length; i++)
            {
                V[i * 2] = VertexArray[i].X;
                V[i * 2 + 1] = VertexArray[i].Y;
            }
            return V;
        }

        // Generic copy into provided array
        public void SequenceToArray<T>(T[] dest, IEnumerable<T> sequence)
        {
            if (dest == null) throw new ArgumentNullException(nameof(dest));
            if (sequence == null) return;

            int i = 0;
            foreach (var s in sequence)
            {
                if (i >= dest.Length) break;
                dest[i++] = s;
            }
        }

    }
}