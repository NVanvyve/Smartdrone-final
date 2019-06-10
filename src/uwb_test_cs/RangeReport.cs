using System;

namespace uwb_test_cs
{
    public struct RangeReport
    {
        public readonly string mid;
        public readonly byte mask;
        public readonly uint[] ranges;
        public readonly ushort nranges;
        public readonly byte rseq;
        public readonly uint rangetime;
        public readonly byte tid;
        public readonly byte aid;

        public RangeReport(string report)
        {
            var splitted = report.Replace("\r", string.Empty)
                .Replace("\n", string.Empty)
                .Split(' ');
            mid = splitted[0];
            if (mid.Length != 2 || splitted.Length != 10)
            {
                throw new ArgumentException($"Malformed message: {report}", nameof(report));
            }

            mask = Convert.ToByte(splitted[1], 16);
            ranges = new uint[4];
            for (var i = 0; i < ranges.Length; i++)
            {
                ranges[i] = Convert.ToUInt32(splitted[2 + i], 16);
            }

            nranges = Convert.ToUInt16(splitted[6], 16);
            rseq = Convert.ToByte(splitted[7], 16);
            rangetime = Convert.ToUInt32(splitted[8], 16);

            tid = Convert.ToByte(splitted[9][1].ToString());
            aid = Convert.ToByte(splitted[9][3].ToString());
        }

        public override string ToString()
        {
            return
                $"MID:{mid} MASK:{mask} RANGES:{string.Join(", ", ranges)} NRANGES:{nranges} RSEQ:{rseq} RANGETIME:{rangetime} TID:{tid} AID:{aid}";
        }

        public bool HasAnchor(byte anchorNumber)
        {
            return ((0x1 << anchorNumber) & mask) > 0;
        }
    }
}