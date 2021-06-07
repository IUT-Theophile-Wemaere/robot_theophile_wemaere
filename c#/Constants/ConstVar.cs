using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Constants
{
    public static class ConstVar
    {
        public const byte START_OF_FRAME = 0xFE;
        public const byte MAX_MSG_LENGHT = 255;

        public const string SCICHART_RUNTIME_KEY = "Wh4gbehyfiMgkJA2zpELG3Z1hR4/RsihR9mWQXQ4oz3z/12i6QyyTgoag5ThU5WmVg3FVgxOAS3stZHqZrMg0btp5DhpXrOEp8zPV/UBZcZO3W9yEzvIuu0odvX0tx8s+EG27UvGlc6qUQXylWIRiG/Pyz9BN4CYd7bHOMQKTEnx6vpexCxfZYK930TYzo2hCJJTaty93ld1feRDCYlGubQMDcV3+9Ogsc+P67ldqTghn5pUZfukASkp3Pw1wui/TPR3iLp1rDTN7RJWzfFkMhTPjDLw46hW18ixWCzWb1F2fI1OghfjcLQFCAVw+v90KsP3V71OaVtW9Ur2WcH0/p0Rlzi+HjmE2rvQkiJjiOr5AoP/tKTM6AmfsCUA3WtHQ/gDpUkkXcadoIZ9YPkt9mR0WdmzsPFMuQ56L38ZpIRVVytLFrPH9yfsDbffsfRMwTU99x9OOgozJorss2WXY4Da8EROiVvcyS1/2UP/KRoYvbTZGrEVe/a/ii4DIhZOvg==";
        public const double EUROBOT_ODOMETRY_POINT_TO_METER = 1.178449e-06;
        public const double EUROBOT_WHEELS_ANGLE = 1.570796e+00;
        public const double EUROBOT_MATRIX_X_COEFF = 5.000000e-01;
        public const double EUROBOT_MATRIX_THETA_COEFF = 4.166667e+00;

        public const double FREQ_IN_HZ = 50d;

        public const string PROJECT_NAME = "Eurobot_Florian_Reimat";
        public const string LOG_FOLDER_NAME = "LogFiles";
        public const long MAX_LOG_FILE_SIZE = 90000000;

        public const int WIDTH_BOXSIZE = 3;
        public const int HEIGHT_BOXSIZE = 2;
    }
}
