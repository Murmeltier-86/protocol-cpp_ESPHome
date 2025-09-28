
using System;
using System.IO.Ports;
using System.Linq;
using System.Xml.Linq;
using System.Text;

class Program
{
    const string PORT_NAME = "COM4";   // anpassen
    const int BAUD = 115200;

    static byte[] EncodeDbWithTerm(byte[] plain)
    {
        byte[] term = new byte[] {0xDF,0xFF,0xDB,0xDB,0xFB,0xFB,0xDB,0xDB};
        byte[] outBuf = new byte[plain.Length * 4 + term.Length];
        int o = 0;
        for (int i = 0; i < plain.Length; i++)
        {
            byte v = plain[i];
            for (int pair = 0; pair < 4; pair++)
            {
                byte sym = 0xDB;
                int shift = 6 - 2*pair;
                byte two = (byte)((v >> shift) & 0x03);
                if ((two & 0x01) != 0) sym |= 0x04;
                if ((two & 0x02) != 0) sym |= 0x20;
                outBuf[o++] = sym;
            }
        }
        foreach (var b in term) outBuf[o++] = b;
        Array.Resize(ref outBuf, o);
        return outBuf;
    }

    static void Main(string[] args)
    {
        string xmlPath = args.Length > 0 ? args[0] : "joe_codes_example.xml";
        var xdoc = XDocument.Load(xmlPath);

        var commands = xdoc.Root!.Elements("command")
            .Select(e => new {
                Name = (string?)e.Attribute("name") ?? "(unnamed)",
                Code = (string?)e.Attribute("code") ?? "",
                Mode = (string?)e.Attribute("mode") ?? "db"
            })
            .Where(c => !string.IsNullOrWhiteSpace(c.Code))
            .ToList();

        Console.WriteLine("Gefundene Kommandos:");
        for (int i=0;i<commands.Count;i++)
            Console.WriteLine($"{i}: {commands[i].Name}  code="{commands[i].Code}"  mode={commands[i].Mode}");

        Console.Write("Index wählen: ");
        if (!int.TryParse(Console.ReadLine(), out int idx) || idx < 0 || idx >= commands.Count) return;

        var cmd = commands[idx];
        bool plain = (cmd.Mode.ToLowerInvariant() == "plain");

        using var sp = new SerialPort(PORT_NAME, BAUD);
        sp.Open();

        if (plain) {
            var data = Encoding.ASCII.GetBytes(cmd.Code);
            sp.Write(data, 0, data.Length);
            Console.WriteLine($"Gesendet (plain): {cmd.Code}");
        } else {
            var data = Encoding.ASCII.GetBytes(cmd.Code);
            var enc = EncodeDbWithTerm(data);
            sp.Write(enc, 0, enc.Length);
            Console.WriteLine($"Gesendet (db, {enc.Length} Bytes): {cmd.Code}");
        }
    }
}
