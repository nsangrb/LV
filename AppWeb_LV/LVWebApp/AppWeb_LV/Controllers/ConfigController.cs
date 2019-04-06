using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;
using System.Web.Mvc;
using AppWeb_LV.Models;
using System.IO;

namespace AppWeb_LV.Controllers
{
    public class ConfigController : Controller
    {
        // GET: Config
        private StreamIO _StreamIO = new StreamIO();
        public ActionResult Stt_RL(string stt = "")
        {
            try
            {
                if (stt != "")
                {
                    _StreamIO.Path = Server.MapPath("../RL.txt");
                    _StreamIO.StreamWriter(stt);
                }
                return View();
            }
            catch
            {
                return View("Error");
            }
        }
        public ActionResult ThresHold(string thres = "")
        {
            try
            {
                if (thres != "")
                {
                    _StreamIO.Path = Server.MapPath("../TH.txt");
                    _StreamIO.StreamWriter(thres);
                }
                return View();
            }
            catch
            {
                return View("Error");
            }
        }
        public ActionResult Config()
        {

            return View();
        }
    }
}