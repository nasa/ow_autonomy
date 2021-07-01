import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtGui, QtCore
from python_qt_binding.QtWidgets import QWidget, QAbstractItemView, QTableWidgetItem

class MyPlugin(Plugin):

	def __init__(self, context):
		super(MyPlugin, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('PLEXILPlanner')

		# Process standalone plugin command-line arguments
		from argparse import ArgumentParser
		parser = ArgumentParser()
		# Add argument(s) to the parser.
		parser.add_argument("-q", "--quiet", action="store_true",
									dest="quiet",
									help="Put plugin in silent mode")
		args, unknowns = parser.parse_known_args(context.argv())
		if not args.quiet:
			print 'arguments: ', args
			print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which should be in the "resource" folder of this package
		ui_file = os.path.join(rospkg.RosPack().get_path('ow_autonomy'), 'resource', 'test.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('MyPluginUi')
		# Show _widget.windowTitle on left-top of each plugin (when 
		# it's set in _widget). This is useful when you open multiple 
		# plugins at once. Also if you open multiple instances of your 
		# plugin at once, these lines add number to make it easy to 
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		rowPosition = self._widget.sentPlansTable.rowCount()
		colPosition = self._widget.sentPlansTable.columnCount()
		self._widget.sentPlansTable.insertRow(rowPosition)
		self._widget.sentPlansTable.insertColumn(colPosition)
		self._widget.sentPlansTable.setItem(rowPosition-1, 1, QTableWidgetItem("text3"))
		self._widget.sentPlansTable.insertRow(rowPosition+1)
		self._widget.sentPlansTable.setItem(rowPosition, 1, QTableWidgetItem("text4"))

		rowPosition = self._widget.planQueueTable.rowCount()
		colPosition = self._widget.planQueueTable.columnCount()
		self._widget.planQueueTable.insertRow(rowPosition)
		self._widget.planQueueTable.insertColumn(colPosition)
		self._widget.planQueueTable.setItem(rowPosition-1, 1, QTableWidgetItem("text3"))
		self._widget.planQueueTable.insertRow(rowPosition+1)
		self._widget.planQueueTable.setItem(rowPosition, 1, QTableWidgetItem("text4"))


		#self._widget.sentPlansTable.insertRow(rowPosition+1)
		#self._widget.sentPlansTable.setItem(rowPosition, 0, QTableWidgetItem("text4"))
		#self._widget.sentPlansTable.setItem(rowPosition, 1, QTableWidgetItem("text5"))

		#self._widget.sentPlansTable.setRowCount(3)
				
		self._widget.planList.setDragDropMode(QAbstractItemView.InternalMove)
		entry = ["Apple", "Orange", "Peach"]
		self._widget.planList.addItems(entry)

		self._widget.removeButton.clicked[bool].connect(self._handle_pushButton_clicked)
		self._widget.upButton.clicked[bool].connect(self._handle_upButton_clicked)

	def _handle_upButton_clicked(self, checked):
		selectedRow = self._widget.planQueueTable.currentRow()
		numRows = self._widget.planQueueTable.rowCount()

		if(selectedRow == 0):
			print("NOPE")
			return
		else:
			belowTxt = self._widget.planQueueTable.item(selectedRow,0).text()
			aboveTxt = self._widget.planQueueTable.item(selectedRow-1,0).text()
			print(belowTxt)
			print(aboveTxt)
			self._widget.planQueueTable.item(selectedRow, 0).setText(aboveTxt)
			self._widget.planQueueTable.item(selectedRow-1, 0).setText(belowTxt)
			return


			
			
		
		
		

	def _handle_pushButton_clicked(self, checked):
		if checked:
			print("TESTING1")
		else:
			print("TESTING2")
			


	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

