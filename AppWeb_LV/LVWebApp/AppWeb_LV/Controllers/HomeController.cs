using System;
using System.Data;
using System.Data.SqlClient;
using System.Web.Mvc;
using AppWeb_LV.Models;

namespace AppWeb_LV.Controllers
{
    public class HomeController : Controller
    {
        private StreamIO _StreamIO = new StreamIO(); 
        public ActionResult Index()
        {
            return View();
        }


        public ActionResult CreateData(string msg="")
        {
            try
            {
                string[] a_str = msg.Split('@');
                int t1, t2, l1, l2;
                float h1, h2;


                t1 = int.Parse(a_str[0]);
                l1 = int.Parse(a_str[1]);
                h1 = float.Parse(a_str[2]);
                t2 = int.Parse(a_str[3]);
                l2 = int.Parse(a_str[4]);
                h2 = float.Parse(a_str[5]);
                DateTime date = DateTime.Now;
                SqlConnection con = new SqlConnection("workstation id=DataModel.mssql.somee.com;packet size=4096;user id=dns2996;pwd=sangngoc;data source=DataModel.mssql.somee.com;persist security info=False;initial catalog=DataModel");
                SqlCommand cmd = new SqlCommand("AddDTNode1", con);
                cmd.CommandType = CommandType.StoredProcedure;
                cmd.Parameters.AddWithValue("@t", t1);
                cmd.Parameters.AddWithValue("@l", l1);
                cmd.Parameters.AddWithValue("@h", h1);
                cmd.Parameters.AddWithValue("@date", date);
                con.Open();
                cmd.ExecuteNonQuery();
                con.Close();

                cmd = new SqlCommand("AddDTNode2", con);
                cmd.CommandType = CommandType.StoredProcedure;
                cmd.Parameters.AddWithValue("@t", t2);
                cmd.Parameters.AddWithValue("@l", l2);
                cmd.Parameters.AddWithValue("@h", h2);
                cmd.Parameters.AddWithValue("@date", date);
                con.Open();
                cmd.ExecuteNonQuery();
                con.Close();
                con.Dispose();
                return View();
            }
           catch
            {
                return View("Error");
            }
        }

        public ActionResult CreateDataAll(string msg="")
        {
            try
            {
                string[] a_str = msg.Split('@');
                int t1, t2, l1, l2;
                float h1, h2;
                string stt;


                t1 = int.Parse(a_str[0]);
                l1 = int.Parse(a_str[1]);
                h1 = float.Parse(a_str[2]);
                t2 = int.Parse(a_str[3]);
                l2 = int.Parse(a_str[4]);
                h2 = float.Parse(a_str[5]);
                stt = a_str[6];

                _StreamIO.Path = Server.MapPath("../RL.txt");
                _StreamIO.StreamWriter(stt);

                DateTime date = DateTime.Now;
                SqlConnection con = new SqlConnection("workstation id=DataModel.mssql.somee.com;packet size=4096;user id=dns2996;pwd=sangngoc;data source=DataModel.mssql.somee.com;persist security info=False;initial catalog=DataModel");
                SqlCommand cmd = new SqlCommand("AddDTNode1", con);
                cmd.CommandType = CommandType.StoredProcedure;
                cmd.Parameters.AddWithValue("@t", t1);
                cmd.Parameters.AddWithValue("@l", l1);
                cmd.Parameters.AddWithValue("@h", h1);
                cmd.Parameters.AddWithValue("@date", date);
                con.Open();
                cmd.ExecuteNonQuery();
                con.Close();

                cmd = new SqlCommand("AddDTNode2", con);
                cmd.CommandType = CommandType.StoredProcedure;
                cmd.Parameters.AddWithValue("@t", t2);
                cmd.Parameters.AddWithValue("@l", l2);
                cmd.Parameters.AddWithValue("@h", h2);
                cmd.Parameters.AddWithValue("@date", date);
                con.Open();
                cmd.ExecuteNonQuery();
                con.Close();
                con.Dispose();
                return View();
            }
            catch
            {
                return View("Error");
            }
        }
    }
}