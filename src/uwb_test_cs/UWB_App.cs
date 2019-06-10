using System;
using System.Collections.Generic;
using IniParser;
using lib_trilateration;
using TagComm;

namespace uwb_test_cs
{
    static class UWB_App
    {
        private const string ConfigFilePath = "./config.ini";

        private static void ReadConfigFile(out string portName, out Coord3D[] anchorArray, string path)
        {
            var parser = new FileIniDataParser();
            var data = parser.ReadFile(path);
            portName = data["tag"]["port"];
             anchorArray = new[]{
                Coord3D.FromString(data["anchors"]["0"]),
                Coord3D.FromString(data["anchors"]["1"]),
                Coord3D.FromString(data["anchors"]["2"])
            };
        }
        
        private static void RangeProcessing(Coord3D[] anchorArray, List<uint> distances)
        {
            Coord3D bestSolution;
            var status = Trilateration.GetLocation(out bestSolution, anchorArray,
                distances.ToArray());
            if (status != Trilateration.Status.TRIL_3SPHERES &&
                status != Trilateration.Status.TRIL_4SPHERES) return;

            Console.Out.WriteLine($"{status}:\t({bestSolution})");
        }

        public static void Main(string[] args)
        {
            Console.WriteLine("*** UWB App ***");

            string portName;
            Coord3D[] anchorArray;
            ReadConfigFile(out portName, out anchorArray, ConfigFilePath);

            Console.WriteLine($"Reading config file at {ConfigFilePath}..");
            Console.WriteLine($"\tUSB port: {portName}");
            Console.WriteLine($"\tAnchor positions : {string.Join(" | ", anchorArray)}");

            // FIXME What if distances are not right order
            // TODO Need to return more than distances: pairs (anchor, distance)
            var tagHandler = new TagHandler(portName);
            tagHandler.Callbacks.Add(distances => RangeProcessing(anchorArray, distances));
            tagHandler.ReadProcess();
        }
    }
}