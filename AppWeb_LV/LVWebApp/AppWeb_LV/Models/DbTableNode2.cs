namespace AppWeb_LV.Models
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel.DataAnnotations;
    using System.ComponentModel.DataAnnotations.Schema;
    using System.Data.Entity.Spatial;

    public partial class DbTableNode2
    {
        public int Id { get; set; }

        public int? Temp { get; set; }

        public int? Light { get; set; }

        public double? Humidity { get; set; }

        public DateTime? DateTime { get; set; }
    }
}
