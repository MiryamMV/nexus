// -----------------------------------------------------------------------------
//  nexus | GenericPhotosensor.h
//
//  Geometry of a configurable box-shaped photosensor.
//
//  The NEXT Collaboration
// -----------------------------------------------------------------------------

#ifndef GENERIC_PHOTOSENSOR_H
#define GENERIC_PHOTOSENSOR_H

#include "BaseGeometry.h"
#include <G4MaterialPropertyVector.hh>

class G4Material;
class G4GenericMessenger;
class G4MaterialPropertiesTable;

namespace nexus {

  class GenericPhotosensor: public BaseGeometry
  {
  public:
    // Constructor for a rectangular sensor providing
    // The default thickness corresponds to a typical value for
    // a silicon photomultiplier.
<<<<<<< HEAD
    GenericPhotosensor(G4String name,   G4double width,
=======
    GenericPhotosensor(G4String name, G4double width,
<<<<<<< HEAD
>>>>>>> Added name to GenericPhotosensor and window optPros bug fixed.
                       G4double height, G4double thickness = 2.0*mm);
=======
                       G4double height, G4double thickness=2.0*mm);
>>>>>>> GenericPhotosensor cosmetic changes on variable names.
    
    // Constructor for a square sensor
    GenericPhotosensor(G4String name, G4double size);
    
    // Destructor
    ~GenericPhotosensor();

    //
    void Construct();

    //
<<<<<<< HEAD
    G4double GetWidth()       const;
    G4double GetHeight()      const;
    G4double GetThickness()   const;
    const G4String& GetName() const;
=======
    G4double GetWidth()     const;
    G4double GetHeight()    const;
    G4double GetThickness() const;
    G4String GetName()      const;
>>>>>>> Added name to GenericPhotosensor and window optPros bug fixed.

    void SetWithWLSCoating       (G4bool with_wls_coating);
    void SetWindowRefractiveIndex(G4MaterialPropertyVector* rindex);
    void SetOpticalProperties    (G4MaterialPropertiesTable* mpt);
    void SetTimeBinning          (G4double time_binning);
    void SetSensorDepth          (G4int sensor_depth);
    void SetMotherDepth          (G4int mother_depth);
    void SetNamingOrder          (G4int naming_order);

  private:

    void ComputeDimensions();
    void DefineMaterials();

    G4GenericMessenger* msg_;

    G4String name_;
    
    G4double width_, height_, thickness_;
<<<<<<< HEAD
    G4double window_thickness_;
    G4double sensarea_width_, sensarea_height_, sensarea_thickness_;
    G4double wls_thickness_;
=======
    G4double sensitive_z_;
    G4double case_x_, case_y_, case_z_;
    G4double window_x_,   window_y_,   window_z_;
    G4double wls_x_,      wls_y_,      wls_z_;
>>>>>>> GenericPhotosensor - Changed 'ENCASING' name by shorter 'CASE'

    G4Material* case_mat_;
    G4Material* window_mat_;
    G4Material* sensitive_mat_;
    G4Material* wls_mat_;

    G4bool                     with_wls_coating_;
    G4MaterialPropertyVector*  window_rindex_;
    G4MaterialPropertiesTable* sensitive_mpt_;

    G4int    sensor_depth_;
    G4int    mother_depth_;
    G4int    naming_order_;
    G4double time_binning_;

    G4bool visibility_;
  };


<<<<<<< HEAD
<<<<<<< HEAD
  inline G4double GenericPhotosensor::GetWidth()       const { return width_; }
  inline G4double GenericPhotosensor::GetHeight()      const { return height_; }
=======
  inline G4double GenericPhotosensor::GetWidth()       const { return case_x_; }
  inline G4double GenericPhotosensor::GetHeight()      const { return case_y_; }
>>>>>>> GenericPhotosensor - Changed 'ENCASING' name by shorter 'CASE'
  inline G4double GenericPhotosensor::GetThickness()   const { return thickness_; }
  inline const G4String& GenericPhotosensor::GetName() const { return name_; }
=======
  inline G4double GenericPhotosensor::GetWidth()     const { return width_; }
  inline G4double GenericPhotosensor::GetHeight()    const { return height_; }
  inline G4double GenericPhotosensor::GetThickness() const { return thickness_; }
  inline G4String GenericPhotosensor::GetName()      const { return name_; }
>>>>>>> Added name to GenericPhotosensor and window optPros bug fixed.

  inline void GenericPhotosensor::SetWithWLSCoating(G4bool with_wls_coating)
  { with_wls_coating_ = with_wls_coating; }

  inline void GenericPhotosensor::SetWindowRefractiveIndex(G4MaterialPropertyVector* rindex)
  { window_rindex_ = rindex; }

  inline void GenericPhotosensor::SetOpticalProperties(G4MaterialPropertiesTable* mpt)
  { sensitive_mpt_ = mpt; }

  inline void GenericPhotosensor::SetTimeBinning(G4double time_binning)
  { time_binning_ = time_binning; }

  inline void GenericPhotosensor::SetSensorDepth(G4int sensor_depth)
  { sensor_depth_ = sensor_depth; }

  inline void GenericPhotosensor::SetMotherDepth(G4int mother_depth)
  { mother_depth_ = mother_depth; }

  inline void GenericPhotosensor::SetNamingOrder(G4int naming_order)
  { naming_order_ = naming_order; }


} // namespace nexus

#endif
