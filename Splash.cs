using System;
using System.Drawing;
using System.Reflection;
using System.Windows.Forms;

namespace MissionPlanner
{
    public partial class Splash : Form
    {
        public Splash()
        {
            InitializeComponent();

            label1.Text = "By Denel Aerospace";
            label1.ForeColor = Color.FromArgb(0x00, 0xBF, 0xFF);

            string strVersion = typeof(Splash).GetType().Assembly.GetName().Version.ToString();

            // Denel release version first — that's the number we hand out with a release ZIP.
            // Upstream's build id is kept in brackets so a support request can still identify
            // exactly which Mission Planner build this was compiled from.
            TXT_version.Text = "Version: " + Program.DenelVersion + "  (MP " + Application.ProductVersion + ")";

            Console.WriteLine(strVersion);

            if (Program.Logo != null)
            {
                pictureBox1.BackgroundImage = MissionPlanner.Properties.Resources.bgdark;
                pictureBox1.Image = Program.Logo;
                pictureBox1.Visible = true;
            }

            Console.WriteLine("Splash .ctor");
        }
    }
}