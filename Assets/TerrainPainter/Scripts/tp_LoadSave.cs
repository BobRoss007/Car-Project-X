////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_LoadSave.cs
//      Author:             HOEKKII
//      
//      Description:        N/A
//      
////////////////////////////////////////////////////////////////////////////

using System;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Xml.Serialization;
using UnityEngine;

namespace TerrainPainter
{
    public static class LoadSave
    {
        #region DeleteFile

        /// <summary>
        /// 
        /// </summary>
        /// <param name="directory">Assets/MyFolder/</param>
        /// <param name="name">MyFile</param>
        /// <param name="extention">xml</param>
        public static string DeleteFile(string directory, string name, string extention = "xml")
        {
            try
            {
                if (!Directory.Exists(directory)) { return "File not Found"; }
                File.Delete(directory + name + "." + extention);
            }
            catch (Exception e)
            {
                return e.ToString();
            }
            return "Success";
        }
        #endregion

        #region Save
        /// <summary>
        /// Save object
        /// </summary>
        /// <param name="obj">YourMember</param>
        /// <param name="directory">Assets/MyFolder/</param>
        /// <param name="name">MyFile</param>
        /// <param name="encrypt"></param>
        public static void Save(object obj, string directory, string name, bool encrypt = false)
        {
            Save(obj, directory, name, "", encrypt);
        }
        public static void Save(object obj, string directory, string name, string encrytionKey, bool encrypt = false)
        {
            StreamWriter writer = null;
            try
            {
                // File
                if (!Directory.Exists(directory)) { Directory.CreateDirectory(directory); }
                FileInfo info = new FileInfo(directory + name + ".xml");
                //if (info.Exists) { info.Delete(); }

                // Write
                writer = info.CreateText();

                if (!string.IsNullOrEmpty(encrytionKey) || encrypt) { writer.Write(Encryption.Encrypt(Serialize(obj), encrytionKey)); }
                else { writer.Write(Serialize(obj)); }

                writer.Close();
            }
            catch { if (writer != null) { writer.Close(); } }
        }
        #endregion

        #region Load
        public static T Load<T>(string directory, string name, bool encrypt = false)
        {
            return Load<T>(directory, name, "", encrypt);
        }
        public static T Load<T>(string directory, string name, string encryptionKey, bool encrypt = false)
        {
            StreamReader reader = null;
            string data = string.Empty;
            try
            {
                // Read file
                reader = File.OpenText(directory + name + ".xml");
                data = reader.ReadToEnd();
                reader.Close();

                // Decrypt if needed
                if (!string.IsNullOrEmpty(encryptionKey) || encrypt) { data = Encryption.Decrypt(data, encryptionKey); }

            }
            catch { if (reader != null) { reader.Close(); } return default(T); }

            // Deserialize
            return Deserialize<T>(data);
        }
        #endregion

        public static string Serialize(object obj)
        {
            XmlSerializer serializer = new XmlSerializer(obj.GetType());
            StringWriter writer = new StringWriter();
            serializer.Serialize(writer, obj);
            return writer.ToString();
        }

        public static T Deserialize<T>(string s)
        {
            XmlSerializer serializer = new XmlSerializer(typeof(T));
            TextReader reader = new StringReader(s);
            return (T)serializer.Deserialize(reader);
        }
    }
    public static class Encryption
    {
        // Default key                        
        private const string defaultKey = "0HOekKIi";

        #region Ecrypt
        /// <summary>
        /// Encrypt text with standart key
        /// </summary>
        /// <param name="text">Unencrypted text</param>
        /// <param name="key">Encryption key</param>
        /// <returns>Encrypted text</returns>
        public static string Encrypt(string text, string key = defaultKey)
        {
            return Encrypt(text, ASCIIEncoding.ASCII.GetBytes(KeyValidation(key)));
        }

        /// <summary>
        /// Encrypt text
        /// </summary>
        /// <param name="text">Unencrypted text</param>
        /// <param name="key">Encryption key</param>
        /// <returns>Encrypted text</returns>
        public static string Encrypt(string text, byte[] key)
        {
            if (string.IsNullOrEmpty(text)) { return ""; }

            DESCryptoServiceProvider desCryptoServiceProvider = new DESCryptoServiceProvider();
            MemoryStream memoryStream = new MemoryStream();
            CryptoStream cryptoStream = new CryptoStream(memoryStream, desCryptoServiceProvider.CreateEncryptor(key, key), CryptoStreamMode.Write);
            StreamWriter streamWriter = new StreamWriter(cryptoStream);
            streamWriter.Write(text);
            streamWriter.Flush();
            cryptoStream.FlushFinalBlock();
            streamWriter.Flush();
            return Convert.ToBase64String(memoryStream.GetBuffer(), 0, (int)memoryStream.Length); ;
        }
        #endregion

        #region Decrypt
        /// <summary>
        /// Decrtypt text with standart key
        /// </summary>
        /// <param name="text">Encrypted text</param>
        /// <param name="key">Decryption key</param>
        /// <returns>Decrypted text</returns>
        public static string Decrypt(string text, string key = defaultKey)
        {
            return Decrypt(text, ASCIIEncoding.ASCII.GetBytes(KeyValidation(key)));
        }

        /// <summary>
        /// Decrypt text
        /// </summary>
        /// <param name="text">Encrypted text</param>
        /// <param name="key">Decryption key</param>
        /// <returns>Decrypted text</returns>
        public static string Decrypt(string text, byte[] key)
        {
            if (string.IsNullOrEmpty(text)) { return ""; }

            DESCryptoServiceProvider desCryptoServiceProvider = new DESCryptoServiceProvider();
            MemoryStream memoryStream = new MemoryStream(Convert.FromBase64String(text));
            CryptoStream cryptoStream = new CryptoStream(memoryStream, desCryptoServiceProvider.CreateDecryptor(key, key), CryptoStreamMode.Read);
            StreamReader streamReader = new StreamReader(cryptoStream);
            return streamReader.ReadToEnd();
        }
        #endregion

        /// <summary>
        /// Check and Fix Key
        /// </summary>
        /// <param name="key">encryption key</param>
        /// <returns>fixed key</returns>
        public static string KeyValidation(string key)
        {
            switch (key.Length)
            {
                case 0: return defaultKey;
                case 8: return key;
                default:
                    string prevKey = key;
                    if (key.Length < 8)
                    {
                        while (key.Length < 8)
                        {
                            key += Convert.ToChar(UnityEngine.Random.Range(0, 255));
                        }
                    }
                    else if (key.Length > 8)
                    {
                        key =
                            key[0].ToString() +
                            key[1].ToString() +
                            key[2].ToString() +
                            key[3].ToString() +
                            key[4].ToString() +
                            key[5].ToString() +
                            key[6].ToString() +
                            key[7].ToString();
                    }
                    else { return defaultKey; }

                    Debug.LogWarningFormat("the length of: {0}, is not 8 characters long. The new key = {1}", prevKey, key);
                    return key;
            }
        }
    }
}
