using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Data;
using System.Data.SqlClient;
using System.Linq;
using System.Web;
using System.Web.Mvc;
using AppWeb_LV.Models;

namespace AppWeb_LV.Controllers
{
    public class Node1Controller : Controller
    {
        private DataModel _Db = new DataModel();
        // GET: Data
        public ActionResult Temp()
        {
            try
            {
                ViewBag.DataNode1 = JsonConvert.SerializeObject(_Db.DbTableNode1.ToList(), _jsonSetting);           
                return View();
            }
            catch (System.Data.Entity.Core.EntityException)
            {
                return View("Error");
            }
            catch (System.Data.SqlClient.SqlException)
            {
                return View("Error");
            }
        }
        public ActionResult Light()
        {
            try
            {
                ViewBag.DataNode1 = JsonConvert.SerializeObject(_Db.DbTableNode1.ToList(), _jsonSetting);
                return View();
            }
            catch (System.Data.Entity.Core.EntityException)
            {
                return View("Error");
            }
            catch (System.Data.SqlClient.SqlException)
            {
                return View("Error");
            }
        }
        public ActionResult Humidity()
        {
            try
            {
                ViewBag.DataNode1 = JsonConvert.SerializeObject(_Db.DbTableNode1.ToList(), _jsonSetting);
                return View();
            }
            catch (System.Data.Entity.Core.EntityException)
            {
                return View("Error");
            }
            catch (System.Data.SqlClient.SqlException)
            {
                return View("Error");
            }
        }
        public ActionResult All()
        {
            try
            {
                ViewBag.DataNode1 = JsonConvert.SerializeObject(_Db.DbTableNode1.ToList(), _jsonSetting);
                return View();
            }
            catch (System.Data.Entity.Core.EntityException)
            {
                return View("Error");
            }
            catch (System.Data.SqlClient.SqlException)
            {
                return View("Error");
            }
        }
        public ActionResult DataFromDataBase()
        {
            try
            {
                ViewBag.DataNode1 = JsonConvert.SerializeObject(_Db.DbTableNode1.ToList(), _jsonSetting);

                return View();
            }
            catch (System.Data.Entity.Core.EntityException)
            {
                return View("Error");
            }
            catch (System.Data.SqlClient.SqlException)
            {
                return View("Error");
            }

        }
        public ActionResult CreateData(string msg="")
        {
            try
            {
                string[] a_str = msg.Split('@');
                int t, l;
                float h;


                t = int.Parse(a_str[0]);
                l = int.Parse(a_str[1]);
                h = float.Parse(a_str[2]);
                DateTime date = DateTime.Now;
                SqlConnection con = new SqlConnection("workstation id=DataModel.mssql.somee.com;packet size=4096;user id=dns2996;pwd=sangngoc;data source=DataModel.mssql.somee.com;persist security info=False;initial catalog=DataModel");
                SqlCommand cmd = new SqlCommand("AddDTNode1", con);
                cmd.CommandType = CommandType.StoredProcedure;
                cmd.Parameters.AddWithValue("@t", t);
                cmd.Parameters.AddWithValue("@l", l);
                cmd.Parameters.AddWithValue("@h", h);
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
        JsonSerializerSettings _jsonSetting = new JsonSerializerSettings() { NullValueHandling = NullValueHandling.Ignore };
    }
}