using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;
using System.IO;

namespace AppWeb_LV.Models
{
    public class StreamIO
    {
        private string path;
        public StreamIO(string path="")
        {
            this.path = path;
        }

        public void StreamWriter(string path, string data, bool append = false)
        {
            StreamWriter wr = new StreamWriter(path, append);
            wr.Flush();
            wr.Write(data);
            wr.Close();
        }
        public void StreamWriter(string data, bool append = false)
        {
            StreamWriter wr = new StreamWriter(path, append);
            wr.Flush();
            wr.Write(data);
            wr.Close();
        }
        public string StreamReader(string path)
        {
            string s;
            StreamReader SR = new StreamReader(path);
            s = SR.ReadLine();
            SR.Close();
            return s;
        }
        public string StreamReader()
        {
            string s;
            StreamReader SR = new StreamReader(path);
            s = SR.ReadLine();
            SR.Close();
            return s;
        }
        public string Path
        {
            get { return path; }
            set { path = value; }
        }
    }
}