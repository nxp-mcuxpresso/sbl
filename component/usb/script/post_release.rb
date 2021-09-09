
# post_release.rb -r D:\test\rel6\7d62fSDK_2.0_LPCXpresso54608 -v 1.6.3 -b lpcxpresso54608

require "fileutils"

$BOARD = 'frdmk66f'
$root_dir = ""

def update_file(path, regular, replace)
    if File.exist?(path)
        File.open(path) do |fr|
            line = fr.read.gsub(regular, replace)
            File.open(path, "w") { |fw| fw.write(line) }
        end
        return true
    end
    return false
end

#update the USB stack version
def update_usb_stack_version()
    if ARGV.include?("-v")
        param = ARGV[ARGV.index("-v") + 1]
        if param.match(/[0-9]+\.[0-9]+\.[0-9]+/)
            regular = /([0-9]+)\.([0-9]+)\.([0-9]+)/
            regular.match(param)
            major = $1
            minor = $2
            bugfix = $3
            # puts "\r\nThe new version of USB stack is #{major}.#{minor}.#{bugfix}"
            # puts "#{$root_dir}/middleware/usb_#{param}/include/usb.h"
            if File.exist?("#{$root_dir}/middleware/usb/include/usb.h")
                update_file("#{$root_dir}/middleware/usb/include/usb.h",/USB_STACK_VERSION_MAJOR *\(.*\)/, "USB_STACK_VERSION_MAJOR \(#{major}U\)");
                update_file("#{$root_dir}/middleware/usb/include/usb.h",/USB_STACK_VERSION_MINOR *\(.*\)/, "USB_STACK_VERSION_MINOR \(#{minor}U\)");
                update_file("#{$root_dir}/middleware/usb/include/usb.h",/USB_STACK_VERSION_BUGFIX *\(.*\)/, "USB_STACK_VERSION_BUGFIX \(#{bugfix}U\)");
                # puts "The USB Stack version is updated!"
            else
                puts "Can't open the file #{$root_dir}/middleware/usb_#{param}/include/usb.h"
            end
            return
        end
    end
    puts("Invalid version parameter!")
end

def update_meta_file(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_meta_file("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if (sub.include?("meta"))
                        # puts "Updating #{path}/#{sub}"
                        param = ARGV[ARGV.index("-v") + 1]
                        if param.match(/[0-9]+\.[0-9]+\.[0-9]+/)
                            regular = /([0-9]+)\.([0-9]+)\.([0-9]+)/
                            regular.match(param)
                            major = $1
                            minor = $2
                            bugfix = $3
                            update_file("#{path}/#{sub}",/version=\"[0-9]+\.[0-9]+\.[0-9]+\"/, "version=\"#{major}\.#{minor}\.#{bugfix}\"");
                        end
                    end
                end
            end
        end
    end
end

#update the USB stack meta file version
def update_usb_stack_meta_file_version()
    if ARGV.include?("-v")
        param = ARGV[ARGV.index("-v") + 1]
        if param.match(/[0-9]+\.[0-9]+\.[0-9]+/)
            regular = /([0-9]+)\.([0-9]+)\.([0-9]+)/
            regular.match(param)
            major = $1
            minor = $2
            bugfix = $3
            # puts "\r\nThe new version of USB stack is #{major}.#{minor}.#{bugfix}"
            # puts "#{$root_dir}/middleware/usb_#{param}/*.meta"
            update_meta_file("#{$root_dir}/middleware/usb");
            return
        end
    end
    puts("Invalid version parameter!")
end

def update_config_file_for_frdmk66f(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_frdmk66f("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(1U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(1U\)");
                    end
                end
            end
        end
    end
end

def update_config_file_for_evkmcimx6ul(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_evkmcimx6ul("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(1U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE *\(.*\)/, "USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE \(1U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(2U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_BUFFER_PROPERTY_CACHEABLE *\(.*\)/, "USB_HOST_CONFIG_BUFFER_PROPERTY_CACHEABLE \(1U\)");
                    end
                end
            end
        end
    end
end

def update_config_file_for_evk_k32h844p(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_evk_k32h844p("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(2U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(1U\)");
                    end
                end
            end
        end
    end
end

def update_config_file_for_evkmcimx6ull(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_evkmcimx6ull("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(1U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE *\(.*\)/, "USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE \(1U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(2U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_BUFFER_PROPERTY_CACHEABLE *\(.*\)/, "USB_HOST_CONFIG_BUFFER_PROPERTY_CACHEABLE \(1U\)");
                    end
                end
            end
        end
    end
end

def update_config_file_for_evkmimxrt1050(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_evkmimxrt1050("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(1U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(2U\)");
                    end
                end
            end
        end
    end
end

def update_config_file_for_frdmk28f(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_frdmk28f("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(1U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(1U\)");
                    end
                end
            end
        end
    end
end

def update_config_file_for_lpcxpresso54114(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_lpcxpresso54114("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(1U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(0U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                    end
                end
            end
        end
    end
end
def update_config_file_for_lpcxpresso51u68(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_lpcxpresso51u68("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(1U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(0U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                    end
                end
            end
        end
    end
end
def update_config_file_for_lpcxpresso54608(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_lpcxpresso54608("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if path.include?("usb_lpm_host_hid_mouse")
                    else
                        if path.include?("usb_keyboard2mouse")
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(1U\)");
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        else
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(1U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(0U\)");
                                path = path.gsub("//", "/")
                                if path.include?("usb_device_hid_mouse/bm") or path.include?("usb_device_hid_mouse\\bm")
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                                if (path.include?("suspend"))
                                    # puts "Updating #{path}/#{sub}"
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        end
                    end
                end
            end
        end
    end
end
def update_config_file_for_lpcxpresso54618(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_lpcxpresso54618("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if path.include?("usb_lpm_host_hid_mouse")
                    else
                        if path.include?("usb_keyboard2mouse")
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(1U\)");
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        else
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(1U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(0U\)");
                                path = path.gsub("//", "/")
                                if path.include?("usb_device_hid_mouse/bm") or path.include?("usb_device_hid_mouse\\bm")
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                                if (path.include?("suspend"))
                                    # puts "Updating #{path}/#{sub}"
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        end
                    end
                end
            end
        end
    end
end

def update_config_file_for_lpcxpresso54s608(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_lpcxpresso54s608("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if path.include?("usb_lpm_host_hid_mouse")
                    else
                        if path.include?("usb_keyboard2mouse")
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(1U\)");
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        else
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(1U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(0U\)");
                                path = path.gsub("//", "/")
                                if path.include?("usb_device_hid_mouse/bm") or path.include?("usb_device_hid_mouse\\bm")
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                                if (path.include?("suspend"))
                                    # puts "Updating #{path}/#{sub}"
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        end
                    end
                end
            end
        end
    end
end

def update_config_file_for_lpcxpresso54628(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_lpcxpresso54628("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if path.include?("usb_lpm_host_hid_mouse")
                    else
                        if path.include?("usb_keyboard2mouse")
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(1U\)");
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        else
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(1U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(0U\)");
                                path = path.gsub("//", "/")
                                if path.include?("usb_device_hid_mouse/bm") or path.include?("usb_device_hid_mouse\\bm")
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                                if (path.include?("suspend"))
                                    # puts "Updating #{path}/#{sub}"
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        end
                    end
                end
            end
        end
    end
end

def update_config_file_for_lpcxpresso54018(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_lpcxpresso54018("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if path.include?("usb_lpm_host_hid_mouse")
                    else
                        if path.include?("usb_keyboard2mouse")
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(1U\)");
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        else
                            if "#{sub}" == "usb_device_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(1U\)");
                                update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(0U\)");
                                path = path.gsub("//", "/")
                                if path.include?("usb_device_hid_mouse/bm") or path.include?("usb_device_hid_mouse\\bm")
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                                if (path.include?("suspend"))
                                    # puts "Updating #{path}/#{sub}"
                                    update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                                end
                            elsif "#{sub}" == "usb_host_config.h"
                                # puts "Updating #{path}/#{sub}"
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(1U\)");
                                update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                            end
                        end
                    end
                end
            end
        end
    end
end

def update_pd_config_file_for_lpcxpresso54018_om13585(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_pd_config_file_for_lpcxpresso54018_om13585("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_pd_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/PD_CONFIG_PTN5110_PORT *\(.*\)/, "PD_CONFIG_PTN5110_PORT \(4U\)");
                    end
                end
            end
        end
    end
end

def update_config_file_for_qn908xdk(path)
    if File.directory?(path)
        Dir.entries(path).each do |sub|
            if sub != '.' && sub != '..'
                if File.directory?("#{path}/#{sub}")
                    #puts "[#{sub}]"
                    update_config_file_for_qn908xdk("#{path}/#{sub}")
                else
                    # puts "  |--#{sub}"
                    if "#{sub}" == "usb_device_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_KHCI *\(.*\)/, "USB_DEVICE_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_EHCI *\(.*\)/, "USB_DEVICE_CONFIG_EHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511FS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511FS \(1U\)");
                        update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_LPCIP3511HS *\(.*\)/, "USB_DEVICE_CONFIG_LPCIP3511HS \(0U\)");
                        if (path.include?("suspend"))
                            # puts "Updating #{path}/#{sub}"
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_SELF_POWER *\(.*\)/, "USB_DEVICE_CONFIG_SELF_POWER \(0U\)");
                            update_file("#{path}/#{sub}",/USB_DEVICE_CONFIG_DETACH_ENABLE *\(.*\)/, "USB_DEVICE_CONFIG_DETACH_ENABLE \(1U\)");
                        end
                    elsif "#{sub}" == "usb_host_config.h"
                        # puts "Updating #{path}/#{sub}"
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_KHCI *\(.*\)/, "USB_HOST_CONFIG_KHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_EHCI *\(.*\)/, "USB_HOST_CONFIG_EHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_OHCI *\(.*\)/, "USB_HOST_CONFIG_OHCI \(0U\)");
                        update_file("#{path}/#{sub}",/USB_HOST_CONFIG_IP3516HS *\(.*\)/, "USB_HOST_CONFIG_IP3516HS \(0U\)");
                    end
                end
            end
        end
    end
end

def update_for_board()
    if ARGV.include?("-b")
        param = ARGV[ARGV.index("-b") + 1]
        if param.match(/frdmk66f/)
            $BOARD = 'frdmk66f'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_frdmk66f("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/evkmcimx6ul/)
            $BOARD = 'evkmcimx6ul'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_evkmcimx6ul("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/evkmcimx6ull/)
            $BOARD = 'evkmcimx6ull'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_evkmcimx6ull("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/evk_k32h844p/)
            $BOARD = 'evk_k32h844p'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_evk_k32h844p("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/evkmimxrt1050/)
            $BOARD = 'evkmimxrt1050'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_evkmimxrt1050("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/frdmk28f/)
            $BOARD = 'frdmk28f'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_frdmk28f("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/lpcxpresso54114/)
            $BOARD = 'lpcxpresso54114'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_lpcxpresso54114("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/lpcxpresso51u68/)
            $BOARD = 'lpcxpresso51u68'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_lpcxpresso51u68("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/lpcxpresso54608/)
            $BOARD = 'lpcxpresso54608'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_lpcxpresso54608("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/lpcxpresso54618/)
            $BOARD = 'lpcxpresso54618'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_lpcxpresso54618("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/lpcxpresso54s608/)
            $BOARD = 'lpcxpresso54s608'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_lpcxpresso54s608("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/lpcxpresso54628/)
            $BOARD = 'lpcxpresso54628'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_lpcxpresso54628("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        if param.match(/lpcxpresso54018/)
            $BOARD = 'lpcxpresso54018'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_lpcxpresso54018("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
            $KIT = 'lpcxpresso54018_om13585'
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$KIT}/usb_examples")
                update_pd_config_file_for_lpcxpresso54018_om13585("#{$root_dir}/boards/#{$KIT}/usb_examples")
            else
                puts "Can't open the file #{$root_dir}/boards/#{$KIT}/usb_examples"
            end
        end
        if param.match(/qn908x/)
            $BOARD = case param
            when /qn908xadk/
                'qn908xadk'
            when /qn908xbdk/
                'qn908xbdk'
            else
                'qn908xcdk'
            end
            if File.directory?("#{$root_dir}") and File.directory?("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                update_config_file_for_qn908xdk("#{$root_dir}/boards/#{$BOARD}/usb_examples")
                # puts "The config file for #{$BOARD} is updated!"
            else
                puts "Can't open the file #{$root_dir}/boards/#{$BOARD}/usb_examples"
            end
        end
        return
    end
    puts("Invalid board parameter!")
end

if ARGV.include?("-r")
    $root_dir = ARGV[ARGV.index("-r") + 1]
    # puts $root_dir
    if File.directory?("#{$root_dir}")
        update_usb_stack_version()
        update_usb_stack_meta_file_version()
        update_for_board()
    else
        puts("Can't open the file #{$root_dir}")
    end
else
    puts("Invalid root dir parameter!")
end
