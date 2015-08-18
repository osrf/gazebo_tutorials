#!/usr/bin/env ruby

require 'rexml/document'

xmlInput = File.read("manifest.xml")

doc, categories = REXML::Document.new(xmlInput), []

# parse list of categories
doc.elements.each("content/categories/category") do |e|
  c = {:ref => e.attributes["ref"], :title => e.attributes["title"], :tutorials => []}
  e.elements.each("tutorials/tutorial") do |t|
    c[:tutorials] << t.text
  end
  categories << c
end

# print csv for each category
rows_per_tutorial = 3
categories.each do |c|
  puts c[:title]
  puts "Tutorial name\tAssigned to\tIssues\tSolution\tApproved?\t\OS / Build type"
  c[:tutorials].each do |t|
    rows_per_tutorial.times do
      puts "=HYPERLINK(\"http://gazebosim.org/tutorials?tut=#{t}&cat=#{c[:ref]}\",\"#{t}\")"
    end
  end
end
