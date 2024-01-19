// For examples, see:
// https://thegraybook.vvvv.org/reference/extending/writing-nodes.html#examples

using System.Runtime.InteropServices;
using VL.Lib.Collections;

namespace Dope;

public static class DopeNativeWrapper
{
    [DllImport("BBW4vvvv.dll")]
    private static extern IntPtr ComputeBBW(double[] Vertices, int[] TI, double[] ControlV, int[] PointI, int[] BoneEdgeI, int[] CageEdgeI, int[] binSizes, bool isTetra);//, IntPtr PseudoEdgeI);

    [DllImport("BBW4vvvv.dll")]
    private static extern int ReleaseMemory(IntPtr ptr);

    public static IEnumerable<double> ComputeWeights(IEnumerable<Vector3> vertices, IEnumerable<int> indices, bool is3D, IEnumerable<Vector3> controls,
                                IEnumerable<int> pointHandleIndices, IEnumerable<int> boneEdgeIndices, IEnumerable<int> cageEdgeIndices, bool bind)
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

        if (bind)
        {
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
            
        }
        return w;
    }

    private class Helpers
    {

        public double[] Vector3DSequenceToArray(double[] V, IEnumerable<Vector3> VertexSequence)
        {
            Vector3[] v = VertexSequence.ToArray<Vector3>();

            for (int i = 0; i < v.Count(); i++)
            {
                V[i * 3] = v[i].X;
                V[i * 3 + 1] = v[i].Y;
                V[i * 3 + 2] = v[i].Z;
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

        public void SequenceToArray<T>(T[] I, IEnumerable<T> Sequence)
        {
            Sequence.ToArray<T>();
        }

    }
}