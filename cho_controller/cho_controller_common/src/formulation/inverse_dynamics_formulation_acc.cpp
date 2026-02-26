#include "cho_controller_common/formulation/inverse_dynamics_formulation_acc.hpp"

using namespace cho_controller::common;
using namespace cho_controller::common::math;
using namespace cho_controller::common::tasks;
using namespace cho_controller::common::solver;
using namespace cho_controller::common::contact;
using namespace std;

typedef pinocchio::Data Data;

InverseDynamicsFormulationAccForce::InverseDynamicsFormulationAccForce(const std::string & name, RobotWrapper & robot, bool verbose)
: InverseDynamicsFormulationBase(name, robot, verbose),
  m_data(robot.model()),
  m_baseDynamics(new math::ConstraintEquality("base-dynamics", 6, robot.nv())),
  m_solutionDecoded(false)
{
  m_mobile = false;

  m_t = 0.0;
  if (m_mobile)
    m_v = robot.nv() - 3;     
  else
    m_v = robot.nv();
  m_k = 0;

  m_eq = 0;
  m_in = 0;
  m_hqpData.resize(2);
  m_Jc.setZero(m_k, m_v);

  //m_hqpData[0].push_back(solvers::make_pair<double, std::shared_ptr<ConstraintBase> >(1.0, m_baseDynamics));
}

Data & InverseDynamicsFormulationAccForce::data()
{
  return m_data;
}

unsigned int InverseDynamicsFormulationAccForce::nVar() const
{
  return m_v+m_k;
}

unsigned int InverseDynamicsFormulationAccForce::nEq() const
{
  return m_eq;
}

unsigned int InverseDynamicsFormulationAccForce::nIn() const
{
  return m_in;
}


void InverseDynamicsFormulationAccForce::resizeHqpData()
{
  m_Jc.setZero(m_k, m_v);
  //m_baseDynamics->resize(m_u, m_v+m_k);
  for(HQPData::iterator it=m_hqpData.begin(); it!=m_hqpData.end(); it++)
  {
    for(ConstraintLevel::iterator itt=it->begin(); itt!=it->end(); itt++)
    {
      itt->second->resize(itt->second->rows(), m_v+m_k);
    }
  }
}

template<class TaskLevelPointer>
void InverseDynamicsFormulationAccForce::addTask(TaskLevelPointer tl, double weight, unsigned int priorityLevel)
{
  if(priorityLevel > m_hqpData.size())
    m_hqpData.resize(priorityLevel);
  const ConstraintBase & c = tl->task.getConstraint();
  if(c.isEquality())
  {
    tl->constraint = std::make_shared<ConstraintEquality>(c.name(), c.rows(), m_v+m_k);
    if(priorityLevel==0)
      m_eq += c.rows();
  }
  else //if(c.isInequality())
  {
    tl->constraint = std::make_shared<ConstraintInequality>(c.name(), c.rows(), m_v+m_k);
    if(priorityLevel==0)
      m_in += c.rows();
  }
  m_hqpData[priorityLevel].push_back(make_pair<double, std::shared_ptr<ConstraintBase> >(weight, tl->constraint));
}


bool InverseDynamicsFormulationAccForce::addMotionTask(TaskMotion & taskmotion, double weight, unsigned int priorityLevel, double transition_duration)
{
  assert(weight>=0.0);
  assert(transition_duration>=0.0);
  
  // This part is not used frequently so we can do some tests.
  if (weight<0.0)
    std::cerr << __FILE__ <<  " " << __LINE__ << " "
      << "weight should be positive" << std::endl;

  // This part is not used frequently so we can do some tests.
  if (transition_duration<0.0)
    std::cerr << "transition_duration should be positive" << std::endl;

  auto tl = std::make_shared<TaskLevel>(taskmotion, priorityLevel);
  m_taskMotions.push_back(tl);
  addTask(tl, weight, priorityLevel);

  return true;
}

bool InverseDynamicsFormulationAccForce::updateTaskWeight(
  const std::string & task_name, double weight)
{
  ConstraintLevel::iterator it;
  // do not look into first priority level because weights do not matter there
  for(unsigned int i=1; i<m_hqpData.size(); i++)
  {
    for(it=m_hqpData[i].begin(); it!=m_hqpData[i].end(); it++)
    {
      if(it->second->name() == task_name)
      {
        it->first = weight;
        return true;
      }
    }
  }
  return false;
}

const HQPData & InverseDynamicsFormulationAccForce::computeProblemData(double time, ConstRefVector q, ConstRefVector v)
{
  m_t = time;
  
  // for(auto it_ct=m_contactTransitions.begin(); it_ct!=m_contactTransitions.end(); it_ct++)
  // {
  //   auto c = *it_ct;
  //   assert(c->time_start <= m_t);
  //   if(m_t <= c->time_end)
  //   {
  //     const double alpha = (m_t - c->time_start) / (c->time_end - c->time_start);
  //     const double fMax = c->fMax_start + alpha*(c->fMax_end - c->fMax_start);
  //     c->contactLevel->contact.setMaxNormalForce(fMax);
  //   }
  //   else
  //   {
  //     // std::cout<<"[InverseDynamicsFormulationAccForce] Remove contact "<<
  //     //            c->contactLevel->contact.name()<<" at time "<<time<<std::endl;
  //     removeRigidContact(c->contactLevel->contact.name());
  //     // FIXME: this won't work if multiple contact transitions occur at the same time
  //     // because after erasing an element the iterator is invalid
  //     m_contactTransitions.erase(it_ct);
  //     break;
  //   }
  // }

  m_robot.computeAllTerms(m_data, q, v);

  // for(auto cl : m_contacts)
  // {
  //   unsigned int m = cl->contact.n_force();

  //   const ConstraintBase & mc = cl->contact.computeMotionTask(time, q, v, m_data);
  //   cl->motionConstraint->matrix().leftCols(m_v) = mc.matrix();
  //   cl->motionConstraint->vector() = mc.vector();

  //   const Matrix & T = cl->contact.getForceGeneratorMatrix(); // e.g., 6x12 for a 6d contact
  //   m_Jc.middleRows(cl->index, m).noalias() = T.transpose()*mc.matrix();

  //   const ConstraintInequality & fc = cl->contact.computeForceTask(time, q, v, m_data);
  //   cl->forceConstraint->matrix().middleCols(m_v+cl->index, m) = fc.matrix();
  //   cl->forceConstraint->lowerBound() = fc.lowerBound();
  //   cl->forceConstraint->upperBound() = fc.upperBound();

  //   const ConstraintEquality & fr = cl->contact.computeForceRegularizationTask(time, q, v, m_data);
  //   cl->forceRegTask->matrix().middleCols(m_v+cl->index, m) = fr.matrix();
  //   cl->forceRegTask->vector() = fr.vector();
  // }

  // ggory15 
  for (auto& it : m_taskMotions)
  {
    const ConstraintBase & c = it->task.compute(time, q, v, m_data);
    
    if(c.isEquality())
    {
      it->constraint->matrix().leftCols(m_v) = c.matrix();
      it->constraint->vector() = c.vector();
    }
    else if(c.isInequality())
    {
      it->constraint->matrix().leftCols(m_v) = c.matrix();
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    }
    else
    {
      it->constraint->matrix().leftCols(m_v) = Matrix::Identity(m_v, m_v);
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    }
  }

  // for (auto& it : m_taskContactForces)
  // {
  //   // cout<<"Task "<<it->task.name()<<endl;
  //   // by default the task is associated to all contact forces
  //   int i0 = m_v;
  //   int c_size = m_k;

  //   // if the task is associated to a specific contact
  //   // cout<<"Associated contact name: "<<it->task.getAssociatedContactName()<<endl;
  //   if(it->task.getAssociatedContactName()!="")
  //   {
  //     // look for the associated contact
  //     for(auto cl : m_contacts)
  //     {
  //       if(it->task.getAssociatedContactName() == cl->contact.name())
  //       {
  //         i0 += cl->index;
  //         c_size = cl->contact.n_force();
  //         break;
  //       }
  //     }
  //   }

  //   const ConstraintBase & c = it->task.compute(time, q, v, m_data, &m_contacts);
  //   // cout<<"matrix"<<endl<<c.matrix()<<endl;
  //   // cout<<"vector"<<endl<<c.vector().transpose()<<endl;
  //   // cout<<"i0 "<<i0<<" c_size "<<c_size<<endl;
  //   // cout<<"constraint matrix size: "<<it->constraint->matrix().rows()<<" x "<<it->constraint->matrix().cols()<<endl;

  //   if(c.isEquality())
  //   {
  //     it->constraint->matrix().middleCols(i0, c_size) = c.matrix();
  //     it->constraint->vector() = c.vector();
  //   }
  //   else if(c.isInequality())
  //   {
  //     it->constraint->matrix().middleCols(i0, c_size) = c.matrix();
  //     it->constraint->lowerBound() = c.lowerBound();
  //     it->constraint->upperBound() = c.upperBound();
  //   }
  //   else
  //   {
  //     it->constraint->matrix().middleCols(i0, c_size) = Matrix::Identity(c_size, c_size);
  //     it->constraint->lowerBound() = c.lowerBound();
  //     it->constraint->upperBound() = c.upperBound();
  //   }
  // }
  
  // cout << solver::HQPDataToString(m_hqpData, true) << endl;
  m_solutionDecoded = false;
  
  return m_hqpData;
}



bool InverseDynamicsFormulationAccForce::decodeSolution(const HQPOutput & sol)
{
  if(m_solutionDecoded)
    return true;

  if (!m_mobile){
    const Matrix & M_a = m_robot.mass(m_data);
    const Vector & h_a = m_robot.nonLinearEffects(m_data);
    const Matrix & J_a = m_Jc.rightCols(m_v);
    
    m_dv = sol.x.head(m_v);
    m_f = sol.x.tail(m_k);
    
    m_tau = h_a;
    m_tau.noalias() += M_a * m_dv;
  // m_tau.noalias() -= J_a.transpose() * m_f;
    m_solutionDecoded = true;
    return true;
  }
  else{
    m_tau.setZero(m_v - 2);
    const Matrix & M_a = m_robot.mass(m_data).bottomRows(m_v- 2);
    const Vector & h_a = m_robot.nonLinearEffects(m_data).tail(m_v-2);

    // const Matrix & J_a = m_Jc.rightCols(m_v);
    
    m_dv = sol.x.head(m_v );
    m_f = sol.x.tail(m_k);
  
    m_tau = h_a;
    m_tau.noalias() += M_a * m_dv;
   
  // m_tau.noalias() -= J_a.transpose() * m_f;
    m_solutionDecoded = true;
    return true;
  }
}

const Vector & InverseDynamicsFormulationAccForce::getAccelerations(const HQPOutput & sol)
{
  decodeSolution(sol);
  return m_dv;
}

const Vector & InverseDynamicsFormulationAccForce::getActuatorForces(const HQPOutput & sol)
{
  decodeSolution(sol);
  return m_tau;
}

bool InverseDynamicsFormulationAccForce::removeTask(const std::string & taskName, double )
{
  #ifndef NDEBUG
  bool taskFound = removeFromHqpData(taskName);
  //assert(taskFound);
  #else
  removeFromHqpData(taskName);
  #endif
  
  for(auto it=m_taskMotions.begin(); it!=m_taskMotions.end(); it++)
  {
    if((*it)->task.name()==taskName)
    {
      if((*it)->priority==0)
      {
        if((*it)->constraint->isEquality())
          m_eq -= (*it)->constraint->rows();
        else if((*it)->constraint->isInequality())
          m_in -= (*it)->constraint->rows();
      }
      m_taskMotions.erase(it);
      return true;
    }
  }
 
  return false;
}

void InverseDynamicsFormulationAccForce::resetHqpData(){
  std::vector<std::shared_ptr<TaskLevel> > new_tasks;
  m_taskMotions = new_tasks;
  HQPData new_Data;
  m_hqpData = new_Data;
  m_hqpData.resize(2);
  m_eq = 0;
  m_in = 0;      
}

bool InverseDynamicsFormulationAccForce::removeFromHqpData(const std::string & name)
{
  bool found = false;
  for(HQPData::iterator it=m_hqpData.begin(); !found && it!=m_hqpData.end(); it++)
  {
    for(ConstraintLevel::iterator itt=it->begin(); !found && itt!=it->end(); itt++)
    {
      if(itt->second->name()==name)
      {
        it->erase(itt);
        return true;
      }
    }
  }
  return false;
}

bool InverseDynamicsFormulationAccForce::addForceTask(
  TaskContactForce & task, double weight, unsigned int priorityLevel, double transition_duration)
{
  assert(weight>=0.0);
  assert(transition_duration>=0.0);
  // This part is not used frequently so we can do some tests.
  if (weight<0.0)
    std::cerr << __FILE__ <<  " " << __LINE__ << " "
      << "weight should be positive" << std::endl;

  // This part is not used frequently so we can do some tests.
  if (transition_duration<0.0)
    std::cerr << "transition_duration should be positive" << std::endl;
  auto tl = std::make_shared<TaskLevelForce>(task, priorityLevel);
  m_taskContactForces.push_back(tl);
  addTask(tl, weight, priorityLevel);
  return true;
}

bool InverseDynamicsFormulationAccForce::addRigidContact(
  ContactBase & contact, double force_regularization_weight, double motion_weight, unsigned int motionPriorityLevel)
{
  auto cl = std::make_shared<ContactLevel>(contact);
  cl->index = m_k;
  m_k += contact.n_force();
  m_contacts.push_back(cl);
  resizeHqpData();

  const ConstraintBase & motionConstr = contact.getMotionConstraint();
  cl->motionConstraint = std::make_shared<ConstraintEquality>(contact.name()+"_motion_task", motionConstr.rows(), m_v+m_k);
  m_hqpData[motionPriorityLevel].push_back(solver::make_pair<double, std::shared_ptr<ConstraintBase> >(motion_weight, cl->motionConstraint));

  const ConstraintInequality & forceConstr = contact.getForceConstraint();
  cl->forceConstraint = std::make_shared<ConstraintInequality>(contact.name()+"_force_constraint", forceConstr.rows(), m_v+m_k);
  m_hqpData[0].push_back(solver::make_pair<double, std::shared_ptr<ConstraintBase> >(1.0, cl->forceConstraint));

  const ConstraintEquality & forceRegConstr = contact.getForceRegularizationTask();
  cl->forceRegTask = std::make_shared<ConstraintEquality>(contact.name()+"_force_reg_task", forceRegConstr.rows(), m_v+m_k);
  m_hqpData[1].push_back(solver::make_pair<double, std::shared_ptr<ConstraintBase> >(force_regularization_weight, cl->forceRegTask));

  m_eq += motionConstr.rows();
  m_in += forceConstr.rows();

  return true;
}

bool InverseDynamicsFormulationAccForce::addRigidContact(ContactBase & contact)
{
    std::cout<<"[InverseDynamicsFormulationAccForce] Method addRigidContact(ContactBase) is deprecated. You should use addRigidContact(ContactBase, double) instead.\n";
    return addRigidContact(contact, 1e-5);
}

bool InverseDynamicsFormulationAccForce::updateRigidContactWeights(
  const std::string & contact_name, double force_regularization_weight, double motion_weight)
{
  // update weight of force regularization task
  ConstraintLevel::iterator itt;
  bool force_reg_task_found = false;
  bool motion_task_found = false;
  for(unsigned int i=1; i<m_hqpData.size(); i++)
  {
    for(itt=m_hqpData[i].begin(); itt!=m_hqpData[i].end(); itt++)
    {
      if(itt->second->name() == contact_name+"_force_reg_task")
      {
        if(force_regularization_weight>=0.0)
          itt->first = force_regularization_weight;
        if(motion_task_found || motion_weight < 0.0)
          return true; // If motion_weight is negative, the motion_task will not be modified. The method can return here
        force_reg_task_found = true;
      }
      else if(itt->second->name() == contact_name+"_motion_task")
      {
        if(motion_weight>=0.0)
          itt->first = motion_weight;
        if(force_reg_task_found)
          return true;
        motion_task_found = true;
      }
    }
  }
  return false;
}

const Vector & InverseDynamicsFormulationAccForce::getContactForces(const HQPOutput & sol)
{
  decodeSolution(sol);
  return m_f;
}

Vector InverseDynamicsFormulationAccForce::getContactForces(
  const std::string & name, const HQPOutput & sol)
{
  decodeSolution(sol);
  // for(std::vector<ContactLevel*>::iterator it=m_contacts.begin(); it!=m_contacts.end(); it++)
  for (auto &it : m_contacts)
  {
    if(it->contact.name()==name)
    {
      const int k = it->contact.n_force();
      return m_f.segment(it->index, k);
    }
  }
  return Vector::Zero(0);
}

bool InverseDynamicsFormulationAccForce::getContactForces(
  const std::string & name, const HQPOutput & sol, RefVector f)
{
  decodeSolution(sol);
  for (auto &it : m_contacts)
  {
    if(it->contact.name()==name)
    {
      const int k = it->contact.n_force();
      assert(f.size()==k);
      f = m_f.segment(it->index, k);
      return true;
    }
  }
  return false;
}

bool InverseDynamicsFormulationAccForce::removeRigidContact(
  const std::string & contactName, double transition_duration)
{
  if(transition_duration>0.0)
  {
    for(auto &it : m_contacts)
    {
      if(it->contact.name()==contactName)
      {
        auto transitionInfo = std::make_shared<ContactTransitionInfo>();
        transitionInfo->contactLevel = it;
        transitionInfo->time_start = m_t;
        transitionInfo->time_end = m_t + transition_duration;
        const int k =  it->contact.n_force();
        if(m_f.size() >= it->index+k)
        {
          const Vector f = m_f.segment(it->index, k);
          transitionInfo->fMax_start = it->contact.getNormalForce(f);
        }
        else
        {
          transitionInfo->fMax_start = it->contact.getMaxNormalForce();
        }
        transitionInfo->fMax_end = it->contact.getMinNormalForce() + 1e-3;
        m_contactTransitions.push_back(transitionInfo);
        return true;
      }
    }
    return false;
  }

  bool first_constraint_found = removeFromHqpData(contactName+"_motion_task");
  assert(first_constraint_found);

  bool second_constraint_found = removeFromHqpData(contactName+"_force_constraint");
  assert(second_constraint_found);

  bool third_constraint_found = removeFromHqpData(contactName+"_force_reg_task");
  assert(third_constraint_found);

  bool contact_found = false;
  for(auto it=m_contacts.begin(); it!=m_contacts.end(); it++)
  {
    if((*it)->contact.name()==contactName)
    {
      m_k -= (*it)->contact.n_force();
      m_eq -= (*it)->motionConstraint->rows();
      m_in -= (*it)->forceConstraint->rows();
      m_contacts.erase(it);
      resizeHqpData();
      contact_found = true;
      break;
    }
  }

  int k=0;
  for (auto &it : m_contacts)
  {
    it->index = k;
    k += it->contact.n_force();
  }
  return contact_found && first_constraint_found && second_constraint_found && third_constraint_found;
}
