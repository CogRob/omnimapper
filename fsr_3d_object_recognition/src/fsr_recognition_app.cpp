#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/progress.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageShiftScale.h>
#include <vtkPNGWriter.h>

#include <fsr_threedorlib/fsr_hashdescription.h>
#include <fsr_threedorlib/fsr_recognition.h>

#define FSR_OBJREC_DEBUG 1
#define FSR_OBJREC_VERBOSE 1

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::RangeImagePlanar RangeImage;
typedef RangeImage::Ptr RangeImagePtr;

typedef std::vector<fsr_or::RegSolEntry<pcl::PointXYZRGBA> > RegistrationSolutions;
typedef boost::shared_ptr<RegistrationSolutions> RegistrationSolutionsPtr;

typedef boost::posix_time::ptime Time;

namespace fs = boost::filesystem;

static fsr_or::FSRHashMapDescription<pcl::PointXYZRGBA> hashmapsearch;
static fsr_or::FSRRecognition<pcl::PointXYZRGBA> object_recognizer;

CloudConstPtr cloud_scene;
CloudConstPtr cloud_scene_reduced;
RangeImagePtr range_image;
Eigen::Affine3f sensor_pose;
boost::shared_ptr<fsr_or::ConflictGraph> conflict_graph;
RegistrationSolutionsPtr solutions;

static bool red_sce_done = false;
static bool ran_img_done = false;
static bool con_gra_done = false;
static bool reg_sol_done = false;
static boost::mutex red_sce_mutex;
static boost::mutex ran_img_mutex;
static boost::mutex con_gra_mutex;
static boost::mutex reg_sol_mutex;
static boost::condition_variable red_sce_cond;
static boost::condition_variable ran_img_cond;
static boost::condition_variable con_gra_cond;
static boost::condition_variable reg_sol_cond;

int classifyDatabase (std::string database);
int classifyObjects (std::string scene, std::string scenefolder, std::ofstream &mfile);
void reducedSceneCallback (const CloudConstPtr &scene, CloudPtr &reduced, Time t);
void rangeImageCallback (RangeImagePtr &ri, Eigen::Affine3f pose, Time t);
void conflictGraphCallback (boost::shared_ptr<fsr_or::ConflictGraph> &graph, Time t);
void solutionsCallback (RegistrationSolutionsPtr &sol, Time t);
void saveRangeImgAsPNG (RangeImagePtr &ri, std::string rname);

int main (int argc, char** argv)
{
  std::cout << "this program classifies objects in scenes " << std::endl << std::endl;
  std::string command, modelpath, savepath, model;

  std::cout << "please enter the full directory path and name of the file of object descriptions: " << std::endl;
  std::cin >> command;
  hashmapsearch.readModelsFromFile(command);
  object_recognizer.setDescriptionInfo(hashmapsearch.getMinDistance (),
                                       hashmapsearch.getMaxDistance (),
                                       hashmapsearch.getAngleStep (),
                                       hashmapsearch.getFractionKeptOfOriginalModel (),
                                       hashmapsearch.getAverageModelSize ());
  object_recognizer.setHashMap (hashmapsearch.getHashMap ());
  object_recognizer.setModelSizes (hashmapsearch.getModelSizes ());
  object_recognizer.setModelKeyBox (hashmapsearch.getModelSizes ());

  boost::function<void (const CloudConstPtr&, CloudPtr&, Time)> red_sce_callback = boost::bind (reducedSceneCallback, _1, _2, _3);
  boost::function<void (RangeImagePtr&, Eigen::Affine3f, Time t)> ran_img_callback = boost::bind (rangeImageCallback, _1, _2, _3);
  boost::function<void (boost::shared_ptr<fsr_or::ConflictGraph>&, Time)> con_gra_callback = boost::bind (conflictGraphCallback, _1, _2);
  boost::function<void (RegistrationSolutionsPtr&, Time)> reg_sol_callback = boost::bind (solutionsCallback, _1, _2);
  object_recognizer.setSceneReducedCallback (red_sce_callback);
  object_recognizer.setRangeImageCallback(ran_img_callback);
  object_recognizer.setConflictGraphCallback (con_gra_callback);
  object_recognizer.setRegistrationSolutionsCallback (reg_sol_callback);

  object_recognizer.spin();

  bool done = false;
  while (!done)
  {
    std::cout << "type:" << std::endl;
    std::cout << "    \"all\" to classify objects in a database of point cloud scenes" << std::endl;
    std::cout << "    \"single\" to classify objects in a single point cloud scene" << std::endl;
    std::cout << "    \"quit\" quit program" << std::endl;
    std::cout << "please enter request: ";
    std::cin >> command;
    if (command.compare("all") == 0)
    {
      std::cout << "enter the full directory path of the database: ";
      std::cin >> modelpath;
      std::cout << std::endl;
      if (classifyDatabase (modelpath) != 0)
      {
        std::cout << "something was wrong with your input" << std::endl;
      }
    }
    else if (command.compare ("single") == 0)
    {
      std::cout << "enter the filename of the scene: ";
      std::cin >> model;
      std::cout << std::endl;

      std::stringstream ss;
      std::string fname = model.substr (model.find_last_of ("/") + 1, model.find_last_of ("."));
      ss << "classify_" << fname << ".txt";
      std::ofstream mfile;
      mfile.open (ss.str().c_str ());

      if (classifyObjects(model, "", mfile) != 0)
      {
        std::cout << "something was wrong with your input" << std::endl;
      }
      mfile.close ();
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

int classifyDatabase (std::string database)
{
  int error = 0, match_err = 0;
  int count_proc = 0, count_skip = 0, count_err = 0;
  fs::path databaseroot (database);
  std::string viewname;
  std::ofstream mfile, rfile;

  std::stringstream ss;
  if (fs::is_directory (databaseroot))
  {
    mfile.open ("classificationresults.txt");
    rfile.open ("classificationresults_results_of_run.txt");

    fs::directory_iterator end_iter;
    for (fs::directory_iterator dir_iter (databaseroot); dir_iter != end_iter; ++dir_iter)
    {
      ss.clear ();
      ss.str (std::string ());
      try
      {
        if (fs::is_directory (dir_iter->status ()))
        {
          std::cout << "skipping directory " << dir_iter->path().string () << "this program does not recursively look for files" << std::endl;
          ss << "skipping directory " << dir_iter->path().string () << "this program does not recursively look for files" << "\n";
        }
        else if (fs::is_regular_file (dir_iter->status ()))
        {
          if (dir_iter->path().filename().string().find (".pcd") != std::string::npos)
          {
            std::cout << "classifying " << dir_iter->path().filename().string () << "\n";

            match_err = classifyObjects (dir_iter->path().filename().string (),dir_iter->path().parent_path().string (), mfile);
            if (match_err == 0)
            {
              mfile << "\n";
              ++count_proc;
              std::cout << "    classified " << dir_iter->path().filename().string () << std::endl;
              ss << "    classified " << dir_iter->path().filename().string () << "\n";
            }
            else if (match_err == 1)
            {
              ++count_skip;
              std::cout << "    file " << dir_iter->path().filename().string () << " is being skipped because the scene was not segmented" << std::endl;
              ss << "    file " << dir_iter->path().filename().string () << " is being skipped because the scene was not segmented\n";
            }
          }
          else
          {
            ++count_skip;
            std::cout << "    file " << dir_iter->path().filename().string () << " is being skipped because it is not a .pcd file" << std::endl;
            ss << "    file " << dir_iter->path().filename().string () << " is being skipped because it is not a .pcd file\n";
          }
        }
      }
      catch (const std::exception &ex)
      {
        std::cout << "error while processing " << dir_iter->path().string () << ", this is error number " << ++count_err << std::endl;
        ss << "error while processing " << dir_iter->path().string () << ", this is error number " << count_err << "\n";
      }

      rfile << ss.str ();
    }

  }
  else
  {
    error = 1;
  }

  if (!error)
  {
    std::stringstream ss2;
    ss2 << count_proc << "files were processed\n";
    ss2 << count_skip << "files were skipped for processing\n";
    ss2 << count_err << "errors occured\n\n\n";
    rfile << ss2.str ();
  }

  rfile.close();
  mfile.close();

  return error;
}

int classifyObjects (std::string scene, std::string scenefolder, std::ofstream &mfile)
{
  std::stringstream ss;
  //std::vector<three_d_or::ECNSDescription> matches;

  if (scenefolder.compare("") == 0)
  {
    ss << scene;
  }
  else
  {
    ss << scenefolder << "/" << scene;
  }

  CloudPtr cloud (new Cloud ());
  pcl::io::loadPCDFile (ss.str().c_str (), *cloud);

  mfile << ss.str () << "\n";

  object_recognizer.cloudCallback(cloud);

  {
    boost::mutex::scoped_lock red_sce_lock (red_sce_mutex);
    boost::mutex::scoped_lock ran_img_lock (ran_img_mutex);
    boost::mutex::scoped_lock con_gra_lock (con_gra_mutex);
    boost::mutex::scoped_lock reg_sol_lock (reg_sol_mutex);
    while (!red_sce_done || !ran_img_done || !con_gra_done || !reg_sol_done)
    {
      red_sce_cond.wait (red_sce_lock);
      ran_img_cond.wait (ran_img_lock);
      con_gra_cond.wait (con_gra_lock);
      reg_sol_cond.wait (reg_sol_lock);
    }
    red_sce_done = false;
    ran_img_done = false;
    con_gra_done = false;
    reg_sol_done = false;
  }

  RegistrationSolutions::iterator it;
  for (it = solutions->begin (); it != solutions->end (); ++it)
  {
    fsr_or::RegSolEntry<pcl::PointXYZRGBA> rse (*it);

    mfile << rse.info->print () << "\n";
    mfile << rse.T.format(fsr_or::MatrixDisplayFmt) << "\n\n";
  }

  /// TODO : write coding for adding cloud to solution range image
  RangeImagePtr solution_image (new RangeImage);

  std::string fname = scene.substr (scene.find_last_of ("/") + 1, scene.find_last_of ("."));
  ss.clear ();
  ss.str (std::string ());
  ss << "fsrtest_" << fname << ".png";
  saveRangeImgAsPNG(range_image, ss.str ());

  /*
  ss.clear ();
  ss.str (std::string ());
  saveRangeImgAsPNG(solution_image, ss.str ());
  ss << "fsrtest_" << fname << "_with_objects.png";
  */

  return 0;
}

void saveRangeImgAsPNG (RangeImagePtr &ri, std::string rname)
{
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New ();
  image->SetDimensions (ri->width, ri->height, 1);
  //image->SetNumberOfScalarComponents (1);
  //image->SetScalarTypeToFloat ();
  //image->AllocateScalars ();

  vtkInformation *imginfo = image->GetInformation ();
  vtkDataObject::SetPointDataActiveScalarInfo(imginfo, VTK_FLOAT, 1);
  image->AllocateScalars(imginfo);

  int *dims = image->GetDimensions ();
  for (int y = 0; y < dims[1]; ++y)
  {
    for (int x = 0; x < dims[0]; ++x)
    {
      float *pixel = static_cast<float*> (image->GetScalarPointer (x,y,0));
      pcl::PointWithRange s = ri->getPoint (x,y);
      pixel[0] = s.range;
    }
  }

  float oldRange = image->GetScalarRange()[1] - image->GetScalarRange()[0];
  float newRange = 255.0f;

  vtkSmartPointer<vtkImageShiftScale> shiftScaleFilter = vtkSmartPointer<vtkImageShiftScale>::New();
  shiftScaleFilter->SetOutputScalarTypeToUnsignedChar();
  shiftScaleFilter->SetInputData(image);
  shiftScaleFilter->SetShift(-1.0f * image->GetScalarRange()[0]);
  shiftScaleFilter->SetScale(newRange/oldRange);
  shiftScaleFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(rname.c_str());
  writer->SetInputConnection(shiftScaleFilter->GetOutputPort());
  writer->Write();
}

void reducedSceneCallback (const CloudConstPtr &scene, CloudPtr &reduced, Time t)
{
  boost::lock_guard<boost::mutex> lock (red_sce_mutex);
  cloud_scene = scene;
  cloud_scene_reduced = reduced;
  red_sce_done = true;
  red_sce_cond.notify_one ();
}

void rangeImageCallback (RangeImagePtr &ri, Eigen::Affine3f pose, Time t)
{
  boost::lock_guard<boost::mutex> lock (ran_img_mutex);
  range_image = ri;
  sensor_pose = pose;
  ran_img_done = true;
  ran_img_cond.notify_one ();
}

void conflictGraphCallback (boost::shared_ptr<fsr_or::ConflictGraph> &graph, Time t)
{
  boost::lock_guard<boost::mutex> lock (con_gra_mutex);
  conflict_graph = graph;
  con_gra_done = true;
  con_gra_cond.notify_one ();
}

void solutionsCallback (RegistrationSolutionsPtr &sol, Time t)
{
  boost::lock_guard<boost::mutex> lock (reg_sol_mutex);
  solutions = sol;
  reg_sol_done = true;
  reg_sol_cond.notify_one ();
}


