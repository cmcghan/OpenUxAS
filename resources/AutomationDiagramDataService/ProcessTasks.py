#! /usr/bin/env python

import xml.dom.minidom
import pandas as pd
import glob
import math 	# pi
import re 	# regular expression
# import the conversion module
import LocalCoords


def ProcessAngledAreaSearchTask(angledAreaSearchTaskNode):
	taskID = 0
	elements = angledAreaSearchTaskNode.getElementsByTagName('TaskID')
	if elements and elements[0].childNodes:
		taskID = int(elements[0].firstChild.data)
	label = ''
	elements = angledAreaSearchTaskNode.getElementsByTagName('Label')
	if elements and elements[0].childNodes:
		label = str(elements[0].firstChild.data)
	print('taskID[{0}], label[{1}]'.format(taskID,label))
	searchAreaPoints = []
	startPointNode = angledAreaSearchTaskNode.getElementsByTagName('StartPoint')
	if len(startPointNode):
		pointsNode = startPointNode[0].getElementsByTagName('Location3D')
		if len(pointsNode):
			latitude = 0.0
			longitude = 0.0
			altitude = 0.0
			elements = pointsNode[0].getElementsByTagName('Latitude')
			if elements:
				latitude = float(elements[0].firstChild.data)
			elements = pointsNode[0].getElementsByTagName('Longitude')
			if elements:
				longitude = float(elements[0].firstChild.data)
			elements = pointsNode[0].getElementsByTagName('Altitude')
			if elements:
				altitude = float(elements[0].firstChild.data)
			searchAreaPoints.append([latitude,longitude,altitude])
	searchAreaId = 0
	searchAreaIdNode = angledAreaSearchTaskNode.getElementsByTagName('SearchAreaID')
	if len(searchAreaIdNode):
		searchAreaId = int(searchAreaIdNode[0].firstChild.data)
		# find the area of interest
		isFoundFile = False
		for areaOfInterestFile in glob.glob('AreaOfInterest_Id*'):
			#	AreaOfInterest_Id_100.xml
			fileId = int(re.search(r'\d+',areaOfInterestFile).group())
			if fileId == searchAreaId:
				isFoundFile = True
				docFileId = xml.dom.minidom.parse(areaOfInterestFile)
				if docFileId.hasChildNodes():
					fileNode = docFileId.firstChild
					areaNode = fileNode.getElementsByTagName('Area')
					# print('areaNode[' + str(areaNode[0].nodeName) + ']')
					if len(areaNode):
						circleNode = areaNode[0].getElementsByTagName('Circle')
						polygonNode = areaNode[0].getElementsByTagName('Polygon')
						rectangleNode = areaNode[0].getElementsByTagName('Rectangle')
						# print('searchAreaNode[' + str(searchAreaNode[0].firstChild) + ']')
						searchAreaType = str('Circle')
						if len(circleNode):
							centerPointNode = circleNode[0].getElementsByTagName('CenterPoint')
							latitude = 0
							longitude = 0
							altitude = 0
							radius = 0
							if len(centerPointNode):
								location3DElements = centerPointNode[0].getElementsByTagName('Location3D')
								if location3DElements:
									elements = location3DElements[0].getElementsByTagName('Latitude')
									if elements:
										latitude = float(elements[0].firstChild.data)
									elements = location3DElements[0].getElementsByTagName('Longitude')
									if elements:
										longitude = float(elements[0].firstChild.data)
									elements = location3DElements[0].getElementsByTagName('Altitude')
									if elements:
										altitude = float(elements[0].firstChild.data)
							radiusNode = circleNode[0].getElementsByTagName('Radius')
							if len(radiusNode):
								radius = float(radiusNode[0].firstChild.data)
							centerNorthEast_m = LocalCoords.LatLong_degToNorthEast_m(latitude,longitude)
							heading = 0;
							headingStep = math.pi/18;	# 10 deg steps
							while heading < 2.0*math.pi:
								north_m = radius*math.sin(heading) + centerNorthEast_m[0] 
								east_m = radius*math.cos(heading) + centerNorthEast_m[1] 
								searchPointLatLong = LocalCoords.NorthEast_mToLatLong_deg(north_m,east_m)
								searchAreaPoints.append([searchPointLatLong[0],searchPointLatLong[1],altitude])
								heading = heading + headingStep
						elif len(polygonNode):
							print('WARNING:: Polygon search not implemented!!!')
						elif len(rectangleNode):
							print('WARNING:: Rectangle search not implemented!!!')
						else:
							print('ERROR:: Unknown search area type[' + searchAreaType +'] encountered!!!')
						if not isFoundFile:
							print('ERROR:: AngledAreaSearchTask could not find AreaOfInterest File for Id[' + str(searchAreaId) + '!!!')

	else:
		print('ERROR:: AngledAreaSearchTask could not parse SearchAreaID!!!')

	
	searchAreaLocationPd = pd.DataFrame(data = searchAreaPoints,columns=['latitude','longitude','altitude'])
	return [taskID,label,searchAreaLocationPd]


def ProcessWatchTask(watchTaskNode):
	taskID = 0
	elements = watchTaskNode.getElementsByTagName('TaskID')
	if elements and elements[0].childNodes:
		taskID = int(elements[0].firstChild.data)
	label = ''
	elements = watchTaskNode.getElementsByTagName('Label')
	if elements and elements[0].childNodes:
		label = str(elements[0].firstChild.data)
	print('taskID[{0}], label[{1}]'.format(taskID,label))


	watchedEntityLocation = []
	watchedEntityID = 0
	watchedEntityIDNode = watchTaskNode.getElementsByTagName('WatchedEntityID')
	if len(watchedEntityIDNode):
		watchedEntityID = int(watchedEntityIDNode[0].firstChild.data)
	if watchedEntityID > 0:
		# find the point of interest
		isFoundFile = False
		for entityStateFile in glob.glob('WatchedEntity_Id*'):
			#	WatchedEntity_Id_1002.xml
			fileId = int(re.search(r'\d+',entityStateFile).group())
			if fileId == watchedEntityID:
				isFoundFile = True
				docFileId = xml.dom.minidom.parse(entityStateFile)
				if docFileId.hasChildNodes():
					fileNode = docFileId.firstChild
					locationNode = fileNode.getElementsByTagName('Location')
					if len(locationNode):
						pointsNode = locationNode[0].getElementsByTagName('Location3D')
						if len(pointsNode):
							latitude = 0.0
							longitude = 0.0
							altitude = 0.0
							elements = pointsNode[0].getElementsByTagName('Latitude')
							if elements:
								latitude = float(elements[0].firstChild.data)
							elements = pointsNode[0].getElementsByTagName('Longitude')
							if elements:
								longitude = float(elements[0].firstChild.data)
							elements = pointsNode[0].getElementsByTagName('Altitude')
							if elements:
								altitude = float(elements[0].firstChild.data)
							watchedEntityLocation.append([latitude,longitude,altitude])
						else:
							print('ERROR:: processing [' + entityStateFile +'] !!!')
					else:		
						print('ERROR:: processing [' + entityStateFile +'] !!!')
				else:
					print('ERROR:: processing [' + entityStateFile +'] !!!')
				break
	else:
		watchedEntityLocationNode = patternSearchTaskNode.getElementsByTagName('SearchLocation')
		if len(watchedEntityLocationNode):
			pointsNode = watchedEntityLocationNode[0].getElementsByTagName('Location3D')
			if len(pointsNode):
				latitude = 0.0
				longitude = 0.0
				altitude = 0.0
				elements = pointsNode[0].getElementsByTagName('Latitude')
				if elements:
					latitude = float(elements[0].firstChild.data)
				elements = pointsNode[0].getElementsByTagName('Longitude')
				if elements:
					longitude = float(elements[0].firstChild.data)
				elements = pointsNode[0].getElementsByTagName('Altitude')
				if elements:
					altitude = float(elements[0].firstChild.data)
				watchedEntityLocation.append([latitude,longitude,altitude])

	watchedEntityLocationPd = pd.DataFrame(data = watchedEntityLocation,columns=['latitude','longitude','altitude'])
	# print('RESULT:: ProcessImpactPointSearchTask->watchedEntityLocationPd [' + str(watchedEntityLocationPd) +'] !!!')
	return [taskID,label,watchedEntityLocationPd]

def ProcessImpactPointSearchTask(impactPointSearchTaskNode):
	taskID = 0
	elements = impactPointSearchTaskNode.getElementsByTagName('TaskID')
	if elements and elements[0].childNodes:
		taskID = int(elements[0].firstChild.data)
	label = ''
	elements = impactPointSearchTaskNode.getElementsByTagName('Label')
	if elements and elements[0].childNodes:
		label = str(elements[0].firstChild.data)
	print('taskID[{0}], label[{1}]'.format(taskID,label))


	searchLocation = []
	searchLocationID = 0
	searchLocationIDNode = impactPointSearchTaskNode.getElementsByTagName('SearchLocationID')
	if len(searchLocationIDNode):
		searchLocationID = int(searchLocationIDNode[0].firstChild.data)
	if searchLocationID > 0:
		# find the point of interest
		isFoundFile = False
		for pointOfInterestFile in glob.glob('PointOfInterest_Id*'):
			#	PointOfInterest_Id_102.xml
			fileId = int(re.search(r'\d+',pointOfInterestFile).group())
			if fileId == searchLocationID:
				isFoundFile = True
				docFileId = xml.dom.minidom.parse(pointOfInterestFile)
				if docFileId.hasChildNodes():
					fileNode = docFileId.firstChild
					locationNode = fileNode.getElementsByTagName('Location')
					if len(locationNode):
						pointsNode = locationNode[0].getElementsByTagName('Location3D')
						if len(pointsNode):
							latitude = 0.0
							longitude = 0.0
							altitude = 0.0
							elements = pointsNode[0].getElementsByTagName('Latitude')
							if elements:
								latitude = float(elements[0].firstChild.data)
							elements = pointsNode[0].getElementsByTagName('Longitude')
							if elements:
								longitude = float(elements[0].firstChild.data)
							elements = pointsNode[0].getElementsByTagName('Altitude')
							if elements:
								altitude = float(elements[0].firstChild.data)
							searchLocation.append([latitude,longitude,altitude])
						else:
							print('ERROR:: processing [' + pointOfInterestFile +'] !!!')
					else:		
						print('ERROR:: processing [' + pointOfInterestFile +'] !!!')
				else:
					print('ERROR:: processing [' + pointOfInterestFile +'] !!!')
				break
	else:
		searchLocationNode = impactPointSearchTaskNode.getElementsByTagName('SearchLocation')
		if len(searchLocationNode):
			pointsNode = searchLocationNode[0].getElementsByTagName('Location3D')
			if len(pointsNode):
				latitude = 0.0
				longitude = 0.0
				altitude = 0.0
				elements = pointsNode[0].getElementsByTagName('Latitude')
				if elements:
					latitude = float(elements[0].firstChild.data)
				elements = pointsNode[0].getElementsByTagName('Longitude')
				if elements:
					longitude = float(elements[0].firstChild.data)
				elements = pointsNode[0].getElementsByTagName('Altitude')
				if elements:
					altitude = float(elements[0].firstChild.data)
				searchLocation.append([latitude,longitude,altitude])

	searchLocationPd = pd.DataFrame(data = searchLocation,columns=['latitude','longitude','altitude'])
	# print('RESULT:: ProcessImpactPointSearchTask->searchLocationPd [' + str(searchLocationPd) +'] !!!')
	return [taskID,label,searchLocationPd]


def ProcessImpactLineSearchTask(impactLineSearchTaskNode):
	taskID = 0
	elements = impactLineSearchTaskNode.getElementsByTagName('TaskID')
	if elements and elements[0].childNodes:
		taskID = int(elements[0].firstChild.data)
	label = ''
	elements = impactLineSearchTaskNode.getElementsByTagName('Label')
	if elements and elements[0].childNodes:
		label = str(elements[0].firstChild.data)
	print('taskID[{0}], label[{1}]'.format(taskID,label))

	pointList = []
	lineId = 0
	lineIdNode = impactLineSearchTaskNode.getElementsByTagName('LineID')
	if len(lineIdNode):
		lineId = int(lineIdNode[0].firstChild.data)
		# find the line of interest
		isFoundFile = False
		for lineOfInterestFile in glob.glob('LineOfInterest_Id*'):
			#	LineOfInterest_Id_101.xml
			fileId = int(re.search(r'\d+',lineOfInterestFile).group())
			if fileId == lineId:
				isFoundFile = True
				docFileId = xml.dom.minidom.parse(lineOfInterestFile)
				if docFileId.hasChildNodes():
					fileNode = docFileId.firstChild
					pointListNode = fileNode.getElementsByTagName('Line')
					# print('pointListNode[' + str(pointListNode[0].nodeName) + ']')
					if len(pointListNode):
						pointsNode = pointListNode[0].getElementsByTagName('Location3D')
						for point in pointsNode:
							latitude = 0.0
							longitude = 0.0
							altitude = 0.0
							if point:
								elements = point.getElementsByTagName('Latitude')
								if elements:
									latitude = float(elements[0].firstChild.data)
								elements = point.getElementsByTagName('Longitude')
								if elements:
									longitude = float(elements[0].firstChild.data)
								elements = point.getElementsByTagName('Altitude')
								if elements:
									altitude = float(elements[0].firstChild.data)
							pointList.append([latitude,longitude,altitude])
		if not isFoundFile:
			print('ERROR:: ImpactLineSearchTask could not find LineOfInterest File for Id[' + str(lineId) + '!!!')

	else:
		print('ERROR:: ImpactLineSearchTask could not parse LineID!!!')

	pointListPd = pd.DataFrame(data = pointList,columns=['latitude','longitude','altitude'])
	return [taskID,label,pointListPd]


def ProcessPatternSearchTask(patternSearchTaskNode):
	taskID = 0
	elements = patternSearchTaskNode.getElementsByTagName('TaskID')
	if elements and elements[0].childNodes:
		taskID = int(elements[0].firstChild.data)
	label = ''
	elements = patternSearchTaskNode.getElementsByTagName('Label')
	if elements and elements[0].childNodes:
		label = str(elements[0].firstChild.data)
	print('taskID[{0}], label[{1}]'.format(taskID,label))

	isGoodSearchLocation = False
	latitude = 0.0
	longitude = 0.0
	altitude = 0.0
	searchLocation = []
	searchLocationID = 0
	searchLocationIDNode = patternSearchTaskNode.getElementsByTagName('SearchLocationID')
	if len(searchLocationIDNode):
		searchLocationID = int(searchLocationIDNode[0].firstChild.data)
	if searchLocationID > 0:
		# find the point of interest
		for pointOfInterestFile in glob.glob('PointOfInterest_Id*'):
			#	PointOfInterest_Id_102.xml
			fileId = int(re.search(r'\d+',pointOfInterestFile).group())
			if fileId == searchLocationID:
				isFoundFile = True
				docFileId = xml.dom.minidom.parse(pointOfInterestFile)
				if docFileId.hasChildNodes():
					fileNode = docFileId.firstChild
					locationNode = fileNode.getElementsByTagName('Location')
					if len(locationNode):
						pointsNode = locationNode[0].getElementsByTagName('Location3D')
						if len(pointsNode):
							elements = pointsNode[0].getElementsByTagName('Latitude')
							if elements:
								latitude = float(elements[0].firstChild.data)
							elements = pointsNode[0].getElementsByTagName('Longitude')
							if elements:
								longitude = float(elements[0].firstChild.data)
							elements = pointsNode[0].getElementsByTagName('Altitude')
							if elements:
								altitude = float(elements[0].firstChild.data)
							isGoodSearchLocation = True
						else:
							print('ERROR:: processing [' + pointOfInterestFile +'] !!!')
					else:		
						print('ERROR:: processing [' + pointOfInterestFile +'] !!!')
				else:
					print('ERROR:: processing [' + pointOfInterestFile +'] !!!')
				break
	else:
		searchLocationNode = patternSearchTaskNode.getElementsByTagName('SearchLocation')
		if len(searchLocationNode):
			pointsNode = searchLocationNode[0].getElementsByTagName('Location3D')
			if len(pointsNode):
				elements = pointsNode[0].getElementsByTagName('Latitude')
				if elements:
					latitude = float(elements[0].firstChild.data)
				elements = pointsNode[0].getElementsByTagName('Longitude')
				if elements:
					longitude = float(elements[0].firstChild.data)
				elements = pointsNode[0].getElementsByTagName('Altitude')
				if elements:
					altitude = float(elements[0].firstChild.data)
				isGoodSearchLocation = True

	# ADD the boundary circle
	if isGoodSearchLocation:
		radius = 0
		radiusNode = patternSearchTaskNode.getElementsByTagName('Extent')
		if len(radiusNode):
			radius = float(radiusNode[0].firstChild.data)
		centerNorthEast_m = LocalCoords.LatLong_degToNorthEast_m(latitude,longitude)
		heading = 0;
		headingStep = math.pi/18;	# 10 deg steps
		while heading < 2.0*math.pi:
			north_m = radius*math.sin(heading) + centerNorthEast_m[0] 
			east_m = radius*math.cos(heading) + centerNorthEast_m[1] 
			searchPointLatLong = LocalCoords.NorthEast_mToLatLong_deg(north_m,east_m)
			searchLocation.append([searchPointLatLong[0],searchPointLatLong[1],altitude])
			heading = heading + headingStep

	searchLocationPd = pd.DataFrame(data = searchLocation,columns=['latitude','longitude','altitude'])
	# print('RESULT:: ProcessPatternSearchTask->searchLocationPd [' + str(searchLocationPd) +'] !!!')
	return [taskID,label,searchLocationPd]

def ProcessPointSearchTask(pointSearchTaskNode):
	# PointSearchTask
	# Members inherited from Task: TaskID, Label, EligibleEntities, RevisitRate, Parameters, Priority, Required,
	# Members inherited from SearchTask: DesiredWavelengthBands, DwellTime, GroundSampleDistance,
	# SearchLocation - Point to search 
	# StandoffDistance - Minimum distance that an aircraft must maintain from the point of interest.
	# ViewAngleList - A list of acceptable look-angles for this task. Each wedge is defined relative to true North. To be a valid look angle, a sensor must be looking from a direction within the bounds of the wedge. 
	taskID = 0
	elements = pointSearchTaskNode.getElementsByTagName('TaskID')
	if elements and elements[0].childNodes:
		taskID = int(elements[0].firstChild.data)
	label = ''
	elements = pointSearchTaskNode.getElementsByTagName('Label')
	if elements and elements[0].childNodes:
		label = str(elements[0].firstChild.data)
	print('taskID[{0}], label[{1}]'.format(taskID,label))

	searchLocation = []
	searchLocationNode = pointSearchTaskNode.getElementsByTagName('SearchLocation')
	if len(searchLocationNode):
		pointsNode = searchLocationNode[0].getElementsByTagName('Location3D')
		if len(pointsNode):
			latitude = 0.0
			longitude = 0.0
			altitude = 0.0
			elements = pointsNode[0].getElementsByTagName('Latitude')
			if elements:
				latitude = float(elements[0].firstChild.data)
			elements = pointsNode[0].getElementsByTagName('Longitude')
			if elements:
				longitude = float(elements[0].firstChild.data)
			elements = pointsNode[0].getElementsByTagName('Altitude')
			if elements:
				altitude = float(elements[0].firstChild.data)
			searchLocation.append([latitude,longitude,altitude])
	searchLocationPd = pd.DataFrame(data = searchLocation,columns=['latitude','longitude','altitude'])
	return [taskID,label,searchLocationPd]

def ProcessLineSearchTask(lineSearchTaskNode):
	# LineSearchTask
	# Members inherited from Task: TaskID, Label, EligibleEntities, RevisitRate, Parameters, Priority, Required,
	# Members inherited from SearchTask: DesiredWavelengthBands, DwellTime, GroundSampleDistance,
	# PointList - Line to search
	# ViewAngleList - Defines a list of acceptable look-angles for this task. See the documentation above for details.
	# UseInertialViewAngles - If true, the ViewAngleList specifies inertial (North-East) angles. See documentation above.
	taskID = 0
	elements = lineSearchTaskNode.getElementsByTagName('TaskID')
	if elements and elements[0].childNodes:
		taskID = int(elements[0].firstChild.data)
	label = ''
	elements = lineSearchTaskNode.getElementsByTagName('Label')
	if elements and elements[0].childNodes:
		label = str(elements[0].firstChild.data)
	print('taskID[{0}], label[{1}]'.format(taskID,label))

	pointList = []
	pointListNode = lineSearchTaskNode.getElementsByTagName('PointList')
	# print('pointListNode[' + str(pointListNode[0].nodeName) + ']')
	if len(pointListNode):
		pointsNode = pointListNode[0].getElementsByTagName('Location3D')
		for point in pointsNode:
			latitude = 0.0
			longitude = 0.0
			altitude = 0.0
			if point:
				elements = point.getElementsByTagName('Latitude')
				if elements:
					latitude = float(elements[0].firstChild.data)
				elements = point.getElementsByTagName('Longitude')
				if elements:
					longitude = float(elements[0].firstChild.data)
				elements = point.getElementsByTagName('Altitude')
				if elements:
					altitude = float(elements[0].firstChild.data)
			pointList.append([latitude,longitude,altitude])
	pointListPd = pd.DataFrame(data = pointList,columns=['latitude','longitude','altitude'])
	return [taskID,label,pointListPd]

def ProcessAreaSearchTask(areaSearchTaskNode):
	# Area search task
	# Members inherited from Task: TaskID, Label, EligibleEntities, RevisitRate, Parameters, Priority, Required,
	# Members inherited from SearchTask: DesiredWavelengthBands, DwellTime, GroundSampleDistance,
	# SearchArea - Area to search 
	# ViewAngleList - A list of acceptable look-angles for this task. 
	#					Each wedge is defined relative to true North. 
	#					To be a valid look angle, a sensor must be looking from a direction within the bounds of the wedge. 

	taskID = 0
	elements = areaSearchTaskNode.getElementsByTagName('TaskID')
	if elements and elements[0].childNodes:
		taskID = int(elements[0].firstChild.data)
	label = ''
	elements = areaSearchTaskNode.getElementsByTagName('Label')
	if elements and elements[0].childNodes:
		label = str(elements[0].firstChild.data)
	print('taskID[{0}], label[{1}]'.format(taskID,label))

	searchAreaPoints = []
	areaNode = areaSearchTaskNode.getElementsByTagName('SearchArea')
	# print('areaNode[' + str(areaNode[0].nodeName) + ']')
	if len(areaNode):
		circleNode = areaNode[0].getElementsByTagName('Circle')
		polygonNode = areaNode[0].getElementsByTagName('Polygon')
		rectangleNode = areaNode[0].getElementsByTagName('Rectangle')
		# print('searchAreaNode[' + str(searchAreaNode[0].firstChild) + ']')
		searchAreaType = str('Circle')
		if len(circleNode):
			centerPointNode = circleNode[0].getElementsByTagName('CenterPoint')
			latitude = 0
			longitude = 0
			altitude = 0
			radius = 0
			if len(centerPointNode):
				location3DElements = centerPointNode[0].getElementsByTagName('Location3D')
				if location3DElements:
					elements = location3DElements[0].getElementsByTagName('Latitude')
					if elements:
						latitude = float(elements[0].firstChild.data)
					elements = location3DElements[0].getElementsByTagName('Longitude')
					if elements:
						longitude = float(elements[0].firstChild.data)
					elements = location3DElements[0].getElementsByTagName('Altitude')
					if elements:
						altitude = float(elements[0].firstChild.data)
			radiusNode = circleNode[0].getElementsByTagName('Radius')
			if len(radiusNode):
				radius = float(radiusNode[0].firstChild.data)
			centerNorthEast_m = LocalCoords.LatLong_degToNorthEast_m(latitude,longitude)
			heading = 0;
			headingStep = math.pi/18;	# 10 deg steps
			while heading < 2.0*math.pi:
				north_m = radius*math.sin(heading) + centerNorthEast_m[0] 
				east_m = radius*math.cos(heading) + centerNorthEast_m[1] 
				searchPointLatLong = LocalCoords.NorthEast_mToLatLong_deg(north_m,east_m)
				searchAreaPoints.append([searchPointLatLong[0],searchPointLatLong[1],altitude])
				heading = heading + headingStep
		elif len(polygonNode):
			print('WARNING:: Polygon search not implemented!!!')
		elif len(rectangleNode):
			print('WARNING:: Rectangle search not implemented!!!')
		else:
			print('ERROR:: Unknown search area type[' + searchAreaType +'] encountered!!!')

		searchBoundaryPd = pd.DataFrame(data = searchAreaPoints,columns=['latitude','longitude','altitude'])
	return [taskID,label,searchBoundaryPd]


def ProcessTaskFile(filename):

	searchTask = []
	doc2 = xml.dom.minidom.parse(filename)
	if doc2.hasChildNodes():
		isGoodMessage = True
		taskTypeNode = doc2.firstChild
		taskType = str(taskTypeNode.nodeName)

		if taskType == 'PointSearchTask':
			print('processing PointSearchTask ...')
			taskData = ProcessPointSearchTask(taskTypeNode)
			if len(taskData):
				searchTask.append(taskData)
		elif taskType == 'LineSearchTask':
			print('processing LineSearchTask ...')
			taskData = ProcessLineSearchTask(taskTypeNode)
			if len(taskData):
				searchTask.append(taskData)
		elif taskType == 'AreaSearchTask':
			print('processing AreaSearchTask ...')
			taskData = ProcessAreaSearchTask(taskTypeNode)
			if len(taskData):
				searchTask.append(taskData)
		elif taskType == 'AngledAreaSearchTask':
			print('processing AngledAreaSearchTask ...')
			taskData = ProcessAngledAreaSearchTask(taskTypeNode)
			if len(taskData):
				searchTask.append(taskData)
		elif taskType == 'ImpactPointSearchTask':
			print('processing ImpactPointSearchTask ...')
			taskData = ProcessImpactPointSearchTask(taskTypeNode)
			if len(taskData):
				searchTask.append(taskData)
		elif taskType == 'ImpactLineSearchTask':
			print('processing ImpactLineSearchTask ...')
			taskData = ProcessImpactLineSearchTask(taskTypeNode)
			if len(taskData):
				searchTask.append(taskData)
		elif taskType == 'PatternSearchTask':
			print('processing PatternSearchTask ...')
			taskData = ProcessPatternSearchTask(taskTypeNode)
			if len(taskData):
				searchTask.append(taskData)
		elif taskType == 'WatchTask':
			print('processing WatchTask ...')
			taskData = ProcessWatchTask(taskTypeNode)
			if len(taskData):
				searchTask.append(taskData)
		else:
			print('ERROR:: Unknown task type[' + taskType +'] encountered!!!')
	return searchTask

def main():
	taskArray = []
	for taskFile in glob.glob('Task_Id*'):
		print('')	# add a line return
		print('***loading [{0}] ***'.format(taskFile))
		taskArray.extend(ProcessTaskFile(taskFile))
	taskArrayPd = pd.DataFrame(data = taskArray,columns=['taskID','label','searchBoundaryPd'])
	print('')	# add a line return
	print('*** saving [Tasks.pkl] ***')
	print('')	# add a line return
	taskArrayPd.to_pickle('Tasks.pkl')

if __name__ == '__main__':
    main()
