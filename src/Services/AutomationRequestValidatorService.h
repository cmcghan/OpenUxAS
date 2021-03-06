// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   AutomationRequestValidatorService.h
 * Author: derek
 *
 * Created on Aug 24, 2015, 9:31 AM
 */


#ifndef UXAS_SERVICE_AUTOMATION_REQUEST_VALIDATOR_SERVICE_H
#define UXAS_SERVICE_AUTOMATION_REQUEST_VALIDATOR_SERVICE_H

#include "ServiceBase.h"
#include "CallbackTimer.h"

#include "afrl/cmasi/OperatingRegion.h"
#include "afrl/cmasi/Task.h"
#include "uxas/messages/task/UniqueAutomationRequest.h"

#include <memory>
#include <deque>
#include <unordered_map>
#include <unordered_set>

namespace uxas
{
namespace service
{

/*! \class AutomationRequestValidatorService
 *\brief Checks all elements of automation requests to make sure they are present 
 * before sending out a UniqueAutomationRequest. 
 * 
 * Configuration String: 
 *  <Service Type="AutomationRequestValidatorService" MaxResponseTime_ms="5000"/>
 * 
 * Options:
 *  - MaxResponseTime_ms
 * 
 * Subscribed Messages:
 *  - afrl::cmasi::AutomationRequest
 *  - afrl::impact::ImpactAutomationRequest
 *  - uxas::messages::task::UniqueAutomationResponse
 *  - uxas::messages::task::TaskAutomationRequest
 *  - afrl::cmasi::AirVehicleConfiguration
 *  - afrl::impact::GroundVehicleConfiguration
 *  - afrl::impact::SurfaceVehicleConfiguration
 *  - afrl::cmasi::AirVehicleState
 *  - afrl::impact::GroundVehicleState
 *  - afrl::impact::SurfaceVehicleState
 *  - afrl::cmasi::RemoveTasks
 *  - uxas::messages::task::TaskInitialized
 *  - afrl::cmasi::OperatingRegion
 *  - afrl::cmasi::KeepInZone
 *  - afrl::cmasi::KeepOutZone
 * 
 * Sent Messages:
 *  - uxas::messages::task::TaskAutomationResponse
 *  - afrl::cmasi::AutomationResponse
 *  - afrl::impact::ImpactAutomationResponse
 *  - uxas::messages::task::UniqueAutomationRequest
 *  - afrl::cmasi::ServiceStatus
 * 
 */


class AutomationRequestValidatorService : public ServiceBase
{
public:

    static const std::string&
    s_typeName()
    {
        static std::string s_string("AutomationRequestValidatorService");
        return (s_string);
    };

    static const std::vector<std::string>
    s_registryServiceTypeNames()
    {
        std::vector<std::string> registryServiceTypeNames = {s_typeName()};
        return (registryServiceTypeNames);
    };
    
    static const std::string&
    s_directoryName()
    {
        static std::string s_string("");
        return (s_string);
    };

    static ServiceBase*
    create()
    {
        return new AutomationRequestValidatorService;
    };

    AutomationRequestValidatorService();

    virtual
    ~AutomationRequestValidatorService();

private:

    static
    ServiceBase::CreationRegistrar<AutomationRequestValidatorService> s_registrar;

    /** brief Copy construction not permitted */
    AutomationRequestValidatorService(AutomationRequestValidatorService const&) = delete;

    /** brief Copy assignment operation not permitted */
    void operator=(AutomationRequestValidatorService const&) = delete;

    bool
    configure(const pugi::xml_node& serviceXmlNode) override;

    bool
    initialize() override;

    //bool
    //start() override;

    //bool
    //terminate() override;

    bool
    processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage) override;

    ////////////////////////
    // TIMER CALLBACKS
    /*! \brief this function gets called when the response timer expires */
    void OnResponseTimeout();
    
    bool isCheckAutomationRequestRequirements(const std::shared_ptr<uxas::messages::task::UniqueAutomationRequest>& uniqueAutomationRequest);
    void checkToSendNextRequest();
    
    /*! \brief  this timer is used to track time for the system to respond
     * to automation requests*/
    uint64_t m_responseTimerId{0};
    /*! \brief  parameter indicating the maximum time to wait for a response (in ms)*/
    uint32_t m_maxResponseTime_ms = {5000}; // default: 5000 ms

    enum AutomationRequestType
    {
        AUTOMATION_REQUEST,
        SANDBOX_AUTOMATION_REQUEST,
        TASK_AUTOMATION_REQUEST
    };
    
    // storage
    std::deque< std::shared_ptr<uxas::messages::task::UniqueAutomationRequest> > m_waitingRequests;
    std::shared_ptr<uxas::messages::task::UniqueAutomationRequest> m_waitingForResponse;
    bool m_isAllClear{true};
    std::unordered_map<int64_t, AutomationRequestType> m_sandboxMap;
    std::unordered_map<int64_t, int64_t> m_playId;
    std::unordered_map<int64_t, int64_t> m_solnId;
    
    std::unordered_set<int64_t> m_availableConfigurationEntityIds;
    std::unordered_set<int64_t> m_availableStateEntityIds;
    std::unordered_set<int64_t> m_availableKeepInZoneIds;
    std::unordered_set<int64_t> m_availableKeepOutZoneIds;
    std::unordered_set<int64_t> m_availableAreaOfInterestIds;
    std::unordered_set<int64_t> m_availableLineOfInterestIds;
    std::unordered_set<int64_t> m_availablePointOfInterestIds;
    std::unordered_map<int64_t, std::shared_ptr<afrl::cmasi::OperatingRegion> > m_availableOperatingRegions;
    std::unordered_map<int64_t, std::shared_ptr<afrl::cmasi::Task> > m_availableTasks;
    std::unordered_set<int64_t> m_availableStartedTaskIds;
    
};

}; //namespace service
}; //namespace uxas

#endif /* UXAS_SERVICE_AUTOMATION_REQUEST_VALIDATOR_SERVICE_H */
