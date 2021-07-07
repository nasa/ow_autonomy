import os
import rospy
import actionlib
import rospkg

from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QBrush, QColor
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem, QAbstractScrollArea, QApplication
from ow_plexil.msg import PlannerAction, PlannerGoal, PlannerResult

class PlexilPlanner(Plugin):

  monitor_signal = pyqtSignal(['QString']) 

  def __init__(self, context):
    super(PlexilPlanner, self).__init__(context)
    self.setObjectName('PlexilPlanner')

    # Process standalone plugin command-line arguments
    parser = ArgumentParser()
    parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
    args, unknowns = parser.parse_known_args(context.argv())
    if not args.quiet:
      print('arguments: ', args)
      print('unknowns: ', unknowns)

    # Find resources and Create QWidget
    self._widget = QWidget()
    ui_file = os.path.join(rospkg.RosPack().get_path('ow_plexil'), 'rqt_planner', 'resource', 'plexilplanner.ui')
    loadUi(ui_file, self._widget)
    self._widget.setObjectName('PlexilPlannerUI')
    if context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    context.add_widget(self._widget)

    #action client setup
    self.planner_client = actionlib.SimpleActionClient("selected_plans", PlannerAction)


    self.monitor_signal[str].connect(self.monitor_status)
    
    #populates the plan list
    self.populate_plan_list()
        
    #sets up tables
    self._widget.sentPlansTable.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
    self._widget.planQueueTable.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
    self._widget.sentPlansTable.insertColumn(0)
    self._widget.sentPlansTable.insertColumn(1)
    self._widget.planQueueTable.insertColumn(0)
    
    #sets up event listeners
    self._widget.addButton.clicked[bool].connect(self._handle_addButton_clicked)
    self._widget.removeButton.clicked[bool].connect(self._handle_removeButton_clicked)
    self._widget.upButton.clicked[bool].connect(self._handle_upButton_clicked)
    self._widget.downButton.clicked[bool].connect(self._handle_downButton_clicked)
    self._widget.sendButton.clicked[bool].connect(self._handle_sendButton_clicked)
    self._widget.resetButton.clicked[bool].connect(self._handle_resetButton_clicked)

  def populate_plan_list(self):
    plan_list = []
    #get directory
    plan_dir = os.path.join(rospkg.RosPack().get_path('ow_plexil'), 'src', 'plans')
    #get all plans with extension .ple
    for i in os.listdir(plan_dir):
      if(i.endswith(".ple")):
        plan_list.append(i.rsplit(".")[0])
    #add to list
    self._widget.planList.addItems(plan_list)

  def done_callback(self,status, result):
    print(result)

  def monitor_status(self, feedback):
    num_rows = self._widget.sentPlansTable.rowCount()
    if(feedback == "COMPLETE"):
      for i in range(num_rows):
        current_status = self._widget.sentPlansTable.item(i,1).text()
        if(current_status == "Running..."):
          self._widget.sentPlansTable.item(i, 1).setText("Finished")
          self._widget.sentPlansTable.item(i, 1).setBackground(QColor(0,128,0))
          break
    else:
      status = feedback.rsplit(":")[0]
      running_plan = feedback.rsplit(":")[1].rsplit(".")[0]
      for i in range(num_rows):
        plan_name = self._widget.sentPlansTable.item(i,0).text()
        current_status = self._widget.sentPlansTable.item(i,1).text()
        if(plan_name == running_plan and current_status == "Pending..."):
          if(status == "SUCCESS"):
            self._widget.sentPlansTable.item(i, 1).setText("Running...")
            break 
          else:
            self._widget.sentPlansTable.item(i, 1).setText("Failed")
            self._widget.sentPlansTable.item(i, 1).setBackground(QColor(255,0,0))
            break 
    return
   
  def feedback_callback(self, feedback):
    feedback_string = str(feedback.progress)
    self.monitor_signal.emit(feedback_string)

  def _handle_addButton_clicked(self, checked):
    #get selected items
    selected_plans = self._widget.planList.selectedItems()
    #create a new row in the queue table for each selection and insert it
    for i in selected_plans:
      row_position = self._widget.planQueueTable.rowCount()
      self._widget.planQueueTable.insertRow(row_position)
      self._widget.planQueueTable.setItem(row_position-1, 1, QTableWidgetItem(i.text()))
    self._widget.planList.selectionModel().clear()
    self._widget.planQueueTable.resizeColumnsToContents()
    return

  def _handle_removeButton_clicked(self, checked):
    selected_rows = self._widget.planQueueTable.selectedItems()
    for i in selected_rows:
      self._widget.planQueueTable.removeRow(self._widget.planQueueTable.row(i))
    self._widget.planQueueTable.selectionModel().clear()
    return

  def _handle_upButton_clicked(self, checked):
    selected_row = self._widget.planQueueTable.currentRow()

    #checks we are not at top of list
    if(selected_row <= 0):
      return
    #switches the two rows and puts selection on previously selected row
    else:
      belowTxt = self._widget.planQueueTable.item(selected_row,0).text()
      aboveTxt = self._widget.planQueueTable.item(selected_row-1,0).text()
      self._widget.planQueueTable.item(selected_row, 0).setText(aboveTxt)
      self._widget.planQueueTable.item(selected_row-1, 0).setText(belowTxt)
      self._widget.planQueueTable.selectionModel().clear()
      self._widget.planQueueTable.setCurrentItem(self._widget.planQueueTable.item(selected_row-1,0))
      return

  def _handle_downButton_clicked(self, checked):
    selected_row = self._widget.planQueueTable.currentRow()
    num_rows = self._widget.planQueueTable.rowCount()

    #checks we are not at bottom of list
    if(selected_row >= num_rows-1):
      return
    #switches the two rows and puts selection on previously selected row
    else:
      belowTxt = self._widget.planQueueTable.item(selected_row+1,0).text()
      aboveTxt = self._widget.planQueueTable.item(selected_row,0).text()
      self._widget.planQueueTable.item(selected_row+1, 0).setText(aboveTxt)
      self._widget.planQueueTable.item(selected_row, 0).setText(belowTxt)
      self._widget.planQueueTable.selectionModel().clear()
      self._widget.planQueueTable.setCurrentItem(self._widget.planQueueTable.item(selected_row+1,0))
      return

  def _handle_sendButton_clicked(self, checked):
    num_rows = self._widget.planQueueTable.rowCount()
    plans_sent = []
    #creates a new row in the sent table for each item in the queue table and inserts it
    for i in range(num_rows):
      plan_name = self._widget.planQueueTable.item(0,0).text()
      plans_sent.append(str(plan_name + ".plx"))
      self._widget.planQueueTable.removeRow(0)
      row_position = self._widget.sentPlansTable.rowCount()
      self._widget.sentPlansTable.insertRow(row_position)
      self._widget.sentPlansTable.setItem(row_position, 0, QTableWidgetItem(plan_name))
      self._widget.sentPlansTable.setItem(row_position, 1, QTableWidgetItem("Pending..."))
      self._widget.sentPlansTable.resizeColumnsToContents()
    
    # Create goal and send to action server for plan execution
    goal = PlannerGoal(command="ADD", plans=plans_sent)
    self.planner_client.wait_for_server()
    self.planner_client.send_goal(goal, done_cb=self.done_callback, active_cb=None, feedback_cb=self.feedback_callback)
    return

  def _handle_resetButton_clicked(self, checked):
    num_rows = self._widget.sentPlansTable.rowCount()
    #deletes each row pressent in sentplanstable
    for i in range(num_rows):
      self._widget.sentPlansTable.removeRow(0)
    goal = PlannerGoal(command="RESET", plans=[])
    self.planner_client.send_goal(goal, done_cb=self.done_callback, active_cb=None, feedback_cb=self.feedback_callback)
    return

  def shutdown_plugin(self):
    # TODO unregister all publishers here
    pass

