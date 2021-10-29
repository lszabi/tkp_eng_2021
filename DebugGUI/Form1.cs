using System;
using System.Collections.Generic;
using System.Drawing;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text;
using System.Windows.Forms;

namespace DebugGUI
{
	public partial class Form1 : Form
	{
		public struct UdpState
		{
			public UdpClient u;
			public IPEndPoint e;
			public Thread t;
		}

		public struct PlotData
		{
			public string legend;
			public float scale;
			public Color color;
			public float[] values;
		}

		public const int port = 9000;
		public const int plot_size = 600;
		public const int plot_n = 5;
		
		public static UdpState client;
		public static string status = "Waiting...";
		public static PlotData[] plots;
		public static int plot_iter = 0;

		public static void ClientThread()
		{
			while (true)
			{
				byte[] receiveBytes = client.u.Receive(ref client.e);
				string str = Encoding.ASCII.GetString(receiveBytes);
				string[] str_arr = str.Split('|');
				status = str_arr[0];
				for (int i = 0; i < plot_n; i++)
				{
					plots[i].legend = "";
				}
				for (int i = 0; (i < str_arr.Length - 1) && (i < plot_n); i++)
				{
					string[] d = str_arr[i + 1].Split(';');
					plots[i].legend = d[0];
					plots[i].scale = float.Parse(d[1], System.Globalization.CultureInfo.InvariantCulture.NumberFormat);
					plots[i].values[plot_iter] = float.Parse(d[2], System.Globalization.CultureInfo.InvariantCulture.NumberFormat);
				}
				plot_iter = (plot_iter + 1) % plot_size;
			}
		}

		public Form1()
		{
			InitializeComponent();
			DoubleBuffered = true;

			plots = new PlotData[5];
			Color[] plot_colors = { Color.Blue, Color.Orange, Color.Lime, Color.Gold, Color.Purple };
			for (int i = 0; i < plot_n; i++)
			{
				plots[i].legend = "";
				plots[i].scale = 100;
				plots[i].color = plot_colors[i];
				plots[i].values = new float[600];

				for (int k = 0; k < 600; k++)
				{
					plots[i].values[k] = 0;
				}
			}

			client = new UdpState();
			client.e = new IPEndPoint(IPAddress.Any, port);
			client.u = new UdpClient(client.e);

			client.t = new Thread(new ThreadStart(ClientThread));
			client.t.IsBackground = true;
			client.t.Start();
		}

		private void Form1_Paint(object sender, PaintEventArgs e)
		{
			Graphics g = e.Graphics;
			Font f = new Font(DefaultFont.FontFamily, 12, FontStyle.Bold);

			g.Clear(Color.Black);
			g.DrawString(status, f, Brushes.Yellow, 10, 10);

			g.DrawRectangle(Pens.Gray, 10, 220, plot_size, 200);
			g.DrawLine(Pens.LightGray, 11, 320, 8 + plot_size, 320);
			g.DrawLine(Pens.LightGray, 10 + plot_iter, 221, 10 + plot_iter, 419);

			for (int i = 0; i < plot_n; i++)
			{
				if (plots[i].legend != "")
				{
					g.DrawString(plots[i].legend, f, new SolidBrush(plots[i].color), 20 + plot_size, 220 + i * 20);
					for (int x = 0; x < plot_size; x++)
					{
						g.DrawRectangle(new Pen(plots[i].color, 1), 10 + x, 320 - plots[i].values[x] * plots[i].scale, 1, 1);
					}
				}
			}
		}

		private void timer1_Tick(object sender, EventArgs e)
		{
			Invalidate();
		}
	}
}