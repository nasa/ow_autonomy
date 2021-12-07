#The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
#Research and Simulation can be found in README.md in the root directory of
#this repository.

import os
import rospy
import actionlib
import rospkg

from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QBrush, QColor, QIcon
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem, QAbstractScrollArea, QMessageBox, QApplication
from ow_plexil.srv import PlanSelection
from std_msgs.msg import String
import rosservice

class PlexilPlanSelectionGUI(Plugin):

  #sets up our signal for the sub callback
  monitor_signal = pyqtSignal(['QString']) 

  def __init__(self, context):
    '''init initializes the widget and sets up our subscriber, publisher and event handlers'''
    super(PlexilPlanSelectionGUI, self).__init__(context)
    self.setObjectName('PlexilPlanSelectionGUI')

    # Process standalone plugin command-line arguments
    parser = ArgumentParser()
    parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
    args, unknowns = parser.parse_known_args(context.argv())

    # Find resources and Create QWidget
    self._widget = QWidget()
    ui_file = os.path.join(rospkg.RosPack().get_path('ow_plexil'), 'rqt_plexil_plan_selection', 'resource', 'plexil_plan_selection.ui')
    loadUi(ui_file, self._widget)
    self._widget.setObjectName('PlexilPlanSelectionUI')
    if context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    context.add_widget(self._widget)

    #set window icon
    icon = os.path.join(rospkg.RosPack().get_path('ow_plexil'), 'rqt_plexil_plan_selection', 'resource', 'icon.png')
    self._widget.setWindowIcon(QIcon(icon))
    #pub and sub setup
    self.status_publisher = rospy.Publisher('plexil_plan_selection_status', String, queue_size=20)
    self.status_subscriber = rospy.Subscriber('plexil_plan_selection_status', String, self.status_callback)
    #client placeholder
    self.plan_select_client = None
  
    #Qt signal to modify GUI from callback
    self.monitor_signal[str].connect(self.monitor_status)

    #populates the plan list, shows different plans based off of what launch file is running
    if rospy.get_param('owlat_flag', False):
      owlat_plan_dir = os.path.join(rospkg.RosPack().get_path('ow_plexil'), 'src', 'plans', 'owlat_plans')
      self.populate_plan_list(owlat_plan_dir)
    else:
      ow_plan_dir = os.path.join(rospkg.RosPack().get_path('ow_plexil'), 'src', 'plans')
      self.populate_plan_list(ow_plan_dir)
 
    #sets up tables
    self._widget.sentPlansTable.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
    self._widget.planQueueTable.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
    self._widget.sentPlansTable.insertColumn(0)
    self._widget.sentPlansTable.insertColumn(1)
    self._widget.planQueueTable.insertColumn(0)
    #sets up event listeners
    self._widget.addButton.clicked[bool].connect(self.handle_add_button_clicked)
    self._widget.removeButton.clicked[bool].connect(self.handle_remove_button_clicked)
    self._widget.upButton.clicked[bool].connect(self.handle_up_button_clicked)
    self._widget.downButton.clicked[bool].connect(self.handle_down_button_clicked)
    self._widget.sendButton.clicked[bool].connect(self.handle_send_button_clicked)
    self._widget.resetButton.clicked[bool].connect(self.handle_reset_button_clicked)

  def populate_plan_list(self, plan_dir):
    '''Finds all .ple plexil plans in the plans directory and stores them in the widget list.'''
    plan_list = []
    #get all plans with extension .ple
    for p in os.listdir(plan_dir):
      if p.endswith(".ple"):
        plan_list.append(p.rsplit(".")[0])
    #add to list
    self._widget.planList.addItems(plan_list)
    self._widget.planList.sortItems()

  def monitor_status(self, feedback):
    '''Signal from callback calls this function to do the work to avoid threading issued with the GUI,
     changes the status on the sent plans table to match the current plan status. Plan can be Pending,
     Running, Finished or Failed.'''
    num_rows = self._widget.sentPlansTable.rowCount()
    #if completed and previously running we set status as finished
    if feedback == "COMPLETE": 
      for i in range(num_rows):
        current_status = self._widget.sentPlansTable.item(i,1).text()
        if current_status == "Running...":
          self._widget.sentPlansTable.item(i, 1).setText("Finished")
          self._widget.sentPlansTable.item(i, 1).setBackground(QColor(0,128,0))
          break
    else:
      #splitting into plan name and status
      status = feedback.rsplit(":")[0]
      running_plan = feedback.rsplit(":")[1].rsplit(".")[0]
      #we set status to running or Failed depending on status 
      for i in range(num_rows):
        plan_name = self._widget.sentPlansTable.item(i,0).text()
        current_status = self._widget.sentPlansTable.item(i,1).text()
        if plan_name == running_plan and current_status == "Pending...":
          if status == "SUCCESS":
            self._widget.sentPlansTable.item(i, 1).setText("Running...")
            break 
          else:
            self._widget.sentPlansTable.item(i, 1).setText("Failed")
            self._widget.sentPlansTable.item(i, 1).setBackground(QColor(230,38,0))
            break 
    return
   
  def status_callback(self, msg):
    '''Callback from status subscriber. Sends the msg to the function monitor_signal for further 
     processing in order to prevent threading issues in the GUI.'''
    #have to use a signal here or else GUI wont update
    feedback_string = str(msg.data)
    self.monitor_signal.emit(feedback_string)

  def handle_add_button_clicked(self, checked):
    '''When the add button is clicked this function moves any selected items to the plan queue table.'''
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

  def handle_remove_button_clicked(self, checked):
    '''When the remove button is clicked this function deletes any selected items from the plan queue table.'''
    selected_rows = self._widget.planQueueTable.selectedItems()
    for i in selected_rows:
      self._widget.planQueueTable.removeRow(self._widget.planQueueTable.row(i))
    self._widget.planQueueTable.selectionModel().clear()
    return

  def handle_up_button_clicked(self, checked):
    '''When up button is clicked this function moves the selected item up in the queue table.'''
    selected_row = self._widget.planQueueTable.currentRow()
    #checks we are not at top of list
    if selected_row <= 0:
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

  def handle_down_button_clicked(self, checked):
    '''When down button is clicked this function moves the selected item down in the queue table.'''
    selected_row = self._widget.planQueueTable.currentRow()
    num_rows = self._widget.planQueueTable.rowCount()

    #checks we are not at bottom of list
    if selected_row >= num_rows-1 or selected_row < 0:
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

  def handle_send_button_clicked(self, checked):
    '''When send button is clicked any items in the plan queue table are sent (in order) to the sent plans table.
     It then publishes a string array of these plans so that the ow_plexil_plan_selection node can run them 
     sequentially. If the subscriber is not connected a popup box reminding the user to run the ow_plexil node will show up.'''
    if self.check_client_set_up() == False:
      return

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

    # Create msg and send to subscriber for plan execution
    self.plan_select_client("ADD", plans_sent)
    return

  def handle_reset_button_clicked(self, checked):
    '''When reset button is clicked all plans in the sent plans table are deleted. It also publishes a string command 
     RESET letting the ow_plexil_plan_selection node know that any plans in its queue should also be deleted.'''
    if self.check_client_set_up() == False:
      return
    num_rows = self._widget.sentPlansTable.rowCount()
    #deletes each row pressent in sentplanstable
    for i in range(num_rows):
      self._widget.sentPlansTable.removeRow(0)
    self.plan_select_client("RESET", [])
    return

  def check_client_set_up(self):
    '''Checks to see if the client is initialized, if not we check if the server is running before initializing the client.
    If server is not yet running we send a popup informing the user and return False.'''
    #checks to see if we have connected to service yet
    if self.plan_select_client == None:
      #we get list of services and see if any match plexil_plan_selection
      services = rosservice.get_service_list()
      service_running = [i for i in services if "plexil_plan_selection" in i]
      #if none exist we send a popup and return
      if len(service_running) == 0:
        popup = QMessageBox()
        popup.setWindowTitle("ow_plexil service not yet connected")
        popup.setText("ow_plexil plan selection service not connected yet, please make sure the ow_plexil launch file is running.")
        popup.exec_()
        return False
      else:
        #client setup
        rospy.wait_for_service('plexil_plan_selection')
        self.plan_select_client = rospy.ServiceProxy('plexil_plan_selection', PlanSelection)
        return True
    else:
      return True


  def shutdown_plugin(self):
    '''handles shutdown procedures for the plugin, unregisters the publisher and subscriber.'''
    self.status_subscriber.unregister()
    pass

