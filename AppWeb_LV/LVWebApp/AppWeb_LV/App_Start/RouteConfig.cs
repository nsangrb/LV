using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;
using System.Web.Mvc;
using System.Web.Routing;

namespace AppWeb_LV
{
    public class RouteConfig
    {
        public static void RegisterRoutes(RouteCollection routes)
        {
            routes.IgnoreRoute("{resource}.axd/{*pathInfo}");

            routes.MapRoute(
                name: "Default",
                url: "{controller}/{action}/{id}",
                defaults: new { controller = "Home", action = "Index", id = UrlParameter.Optional }
            );
        }
        //public static void RegisterRoutes(RouteCollection routes)
        //{
        //    routes.IgnoreRoute("{resource}.axd/{*pathInfo}");
        //    routes.MapRoute("Default", "{controller}.aspx/{action}/{id}", new { action = "Temp", id = "" });
        //    routes.MapRoute("Root", "", new { controller = "Home", action = "Temp", id = "" });
        //    routes.MapRoute("Root", "Node1/Temp", new { controller = "Node1", action = "Temp", id = "" });
        //}
    }
}
