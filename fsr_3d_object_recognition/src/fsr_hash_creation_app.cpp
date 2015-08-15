#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/progress.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <sstream>

#include <fsr_threedorlib/fsr_hashdescription.h>

#define FSR_OBJDESC_DEBUG 1
#define FSR_OBJDESC_VERBOSE 1

bool describeDatabase (std::string database, bool append);
bool createDescription (std::string model, std::string viewfolder, std::ofstream &dfile);
bool getViewInfo(std::string fname, std::string &class_name, std::string &class_num);

namespace fs = boost::filesystem;

fsr_or::FSRHashMapDescription<pcl::PointXYZ> hashmapsearch;

int main (int argc, char **argv)
{
  std::cout << "this program creates a hashtable of model descriptions from clouds of models " << std::endl;
  std::string command, modelpath, savepath, model;
  bool done = false;
  while (!done)
  {
    std::cout << "type:" << std::endl;
    std::cout << "    \"all\" to use of all views in a database" << std::endl;
    std::cout << "    \"single\" to use a single view" << std::endl;
    std::cout << "    \"quit\" quit program" << std::endl;
    std::cout << "please enter request: ";
    std::cin >> command;
    if (command.compare("all") == 0)
    {
      std::cout << "enter the full directory path of the database: ";
      std::cin >> modelpath;
      std::cout << std::endl;
      if (!describeDatabase (modelpath, false))
      {
        std::cout << "something was wrong with your input" << std::endl;
      }
    }
    else if (command.compare ("single") == 0)
    {
      std::cout << "enter the filename of the model: ";
      std::cin >> model;
      std::cout << std::endl;

      std::stringstream ss;
      std::ofstream file;
      std::string fname = model.substr (model.find_last_of ("/") + 1, model.find_last_of ("."));
      ss << "desc_" << model << ".txt";
      file.open (ss.str().c_str ());

      if (!createDescription (model, "", file))
      {
        std::cout << "something was wrong with your input" << std::endl;
      }
      file.close ();
    }
    else if (command.compare ("quit") == 0)
    {
      done = true;
    }
    else
    {
      std::cout << "please enter a valid command" << std::endl;;
    }
    std::cout << std::endl;
  }

  return 0;
}

bool describeDatabase (std::string database, bool append)
{
  bool error = false;
  int count_proc = 0, count_skip = 0, count_err = 0;
  fs::path databaseroot (database);
  std::string viewname;
  std::ofstream dfile, rfile;

  std::stringstream ss;
  if (fs::is_directory (databaseroot))
  {
    if (append)
    {
      dfile.open ("modeldescriptions.txt", std::ios_base::app);
    }
    else
    {
      dfile.open ("modeldescriptions.txt");
    }
    rfile.open ("modeldescriptions_results_of_run.txt");

    fs::recursive_directory_iterator end_iter;
    for (fs::recursive_directory_iterator dir_iter (databaseroot); dir_iter != end_iter; ++dir_iter)
    {
      ss.clear ();
      ss.str (std::string ());
      try
      {
        if (fs::is_directory (dir_iter->status ()))
        {
          std::cout << "processing directory " << dir_iter->path().string () << "." << std::endl;
          ss << "processing directory " << dir_iter->path().string () << ".\n";
        }
        else if (fs::is_regular_file (dir_iter->status ()))
        {
          if (dir_iter->path().filename().string().find (".pcd") != std::string::npos)
          {
            std::cout << "creating descripton for view " << dir_iter->path().filename().string () << ".\n";

            if (!createDescription (dir_iter->path().filename().string (),dir_iter->path().parent_path().string (), dfile))
            {
              ++count_skip;
              std::cout << "file " << dir_iter->path().filename().string () << " is being skipped." << std::endl;
              ss << "file " << dir_iter->path().filename().string () << " is being skipped.\n";
            }
            else
            {
              dfile << "\n";
              ++count_proc;
              std::cout << "created description for view " << dir_iter->path().filename().string () << "." << std::endl;
              ss << "created description for view " << dir_iter->path().filename().string () << ".\n";
            }
          }
          else
          {
            ++count_skip;
            std::cout << "file " << dir_iter->path().filename().string () << " is being skipped because it is not a .pcd file." << std::endl;
            ss << "file " << dir_iter->path().filename().string () << " is being skipped because it is not a .pcd file\n";
          }
        }
      }
      catch (const std::exception &ex)
      {
        std::cout << "error while processing " << dir_iter->path().string () << ", this is error number " << ++count_err << "." << std::endl;
        ss << "error while processing " << dir_iter->path().string () << ", this is error number " << count_err << ".\n";
      }

      rfile << ss.str ();
    }

  }
  else
  {
    error = true;
  }

  if (!error)
  {
    std::stringstream ss2;
    ss2 << count_proc << "files were processed.\n";
    ss2 << count_skip << "files were skipped for processing.\n";
    ss2 << count_err << "errors occured.\n\n\n";
    rfile << ss2.str ();
  }

  rfile.close();
  dfile.close();

  return !error;
}

bool createDescription (std::string model, std::string viewfolder, std::ofstream &dfile)
{
  std::stringstream ss;
  std::string cfull, cname, cnum;

  if (viewfolder.compare("") == 0)
  {
    cfull = model.substr (model.find_last_of ("/") + 1, model.size ());
    ss << model;
  }
  else
  {
    cfull = model;
    ss << viewfolder << "/" << model;
  }

  if (!getViewInfo(cfull, cname, cnum))
  {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile (ss.str().c_str (), *cloud);

  int icnum = std::stoi (cnum);
  hashmapsearch.addModelToFile (cloud, cname, icnum, dfile);

  return true;
}

bool getViewInfo(std::string fname, std::string &class_name, std::string &class_num)
{
  std::stringstream ss;
  std::string digits = "0123456789";
  size_t i, j, stop;

  i = fname.find ('_');
  if (i == std::string::npos)
  {
    return false;
  }

  /// get class name by adding to the class name until a digit is reached
  stop = fname.find_first_of (digits);
  if (stop == std::string::npos)
  {
    return false;
  }

  i = 0;
  j = 0;
  while (i != stop)
  {
    j = fname.find ('_', i);
    ss << fname.substr (i, j - i);
    i = j + 1;

    //std::cout << ss.str() << std::endl;

    if (ss.str().size () > 100)
    {
      return false;
    }
  }
  class_name = ss.str ();

  ss.clear ();
  ss.str (std::string ());

  /// get class number
  stop = fname.find ('.', i); /// i = first digit in class number
  if (stop == std::string::npos)
  {
    return false;
  }

  ss << fname.substr (i, stop - i); /// i = first digit, stop = '.' before file extension
  //std::cout << ss.str() << std::endl;
  class_num = ss.str ();

  return true;
}
