using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AutomaticWaterMasking
{
    // TODO: add more exceptions here, and stop using generic Exception when appropriate
    public class IncorrectRelationException : Exception
    {
        public IncorrectRelationException()
        {
        }

        public IncorrectRelationException(string message)
        : base(message)
        {
        }

        private static String FormatMessage(Way<AutomaticWaterMasking.Point> way)
        {
            return "Way with id " + way.wayID + " has an invalid relation of '" + way.relation + "'";
        }

        public IncorrectRelationException(Way<AutomaticWaterMasking.Point> way) : this(FormatMessage(way))
        {
        }

    }
}
