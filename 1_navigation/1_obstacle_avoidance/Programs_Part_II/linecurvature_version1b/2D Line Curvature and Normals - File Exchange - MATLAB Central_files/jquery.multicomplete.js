(function($) {

  var Matchers = {
    anywhere: function(text) {
      return $.ui.autocomplete.escapeRegex( text.replace(/[(),]/i, "") );
    },

    start_of_word: function(text) {
      return "(^| )" + $.ui.autocomplete.escapeRegex( text.replace(/[(),]/i, "") );
    }
  };


  $.fn.multicomplete = function(options) {
    options             = options || {};
    options.match_model = options.match_model || "start_of_word";
    options.constrain   = options.constrain   || false;
    options.data_source = options.data_source || get_data_source_from_element(this);
    options.separator   = options.separator   || ",";

    var selected = get_selected_values(this, options.separator);
    var input    = create_input(this);

    input.attr({autocomplete: "off"}).addClass("multicomplete-input");

    if (input.autoscale) { input.autoscale(); }

    initialize_multicomplete(input, selected, options);

    initialize_autocomplete(options.data_source, function(data) {
      input.autocomplete({
        open: function(event, ui) {
          // MathWorks-specific:
          // #footer in sitewide.css has a z-index of 99999999
          $(".ui-autocomplete").css({"z-index": 999999999});
        },

        source: autocomplete_source(
          Matchers[options.match_model], data
        ),

        focus: function(event, ui) {
          input.val(ui.item.label || ui.item.value);
          return false;
        },

        select: function(event, ui) {
          input.trigger("multicomplete.add", ui.item);
          return false;
        }
      });
    });

    return input;
  };


  var initialize_multicomplete = function(input, selected, options) {
    wrap_multicomplete(input);

    bind_multicomplete_listeners(input, options.constrain);

    input.closest("form").submit(function(event) {
      input.trigger("multicomplete.add");
      return true;
    });

    $.each(selected, function(index, item) {
      input.trigger("multicomplete.add", item);
    });
  };

  var wrap_multicomplete = function(input) {
    $("<div/>").addClass("multicomplete")
      .insertBefore(input).append(input)
      .click(function() {
        input.focus();
        return false;
      });
  };

  var bind_multicomplete_listeners = function(input, constrain) {
    if (!constrain) {
      input.bind("keyup", function(event) {
        if (event.keyCode == "188") {  // 188 -> comma
          $(this).trigger("multicomplete.add", {
            value: $(this).val().slice(0, $(this).val().indexOf(","))
          });
        }
      });
    }

    input.bind("multicomplete.add", function(event, item) {
      var $this = $(this);

      if (item == undefined && constrain) { return false }

      item       = item || { value: $this.val() };
      item.value = $.trim(item.value);

      if (item.value == "") { return false }

      var exists = $(":input[value='" + item.value + "'][name='" + $this.data("name") + "']");
      if (exists.size() > 0) {
        $this.val("");
        return false;
      }

      if (item.value.indexOf(",") > -1) {
        $.each(item.value.split(","), function(_value) {
          $this.trigger("add", {value: _value});
        })
        return false;
      }

      create_label($this, item).insertBefore($this);
      $this.val("");

      return true;
    });
  };

  var create_label = function(input, item) {
    return $("<div/>")
      .addClass("multicomplete-selected-item")
      .append($("<span/>")
        .text(item.label || item.value))
      .append($("<input/>")
        .attr({type: "hidden", name: input.data("name")})
        .val(item.value))
      .append($("<a/>")
        .attr({href: "#remove", tabIndex: "-1", rel: "nofollow"})
        .text("remove")
        .click(function() {
          $(this).closest("div").remove();
          input.focus();
          return false;
        }));
  };

  var autocomplete_source = function(matcher, data) {
    return function(request, response) {
      var regexp = new RegExp(matcher(request.term), "i");

      response($.grep(data, function(value) {
        value = value.label || value.value || value;
        return regexp.test(value);
      }));
    }
  };

  var get_selected_values = function(element, separator) {
    if (element.is("select")) {
      return map_options_to_items(element.children(":selected"));
    } else if (element.val()) {
      return $.map(element.val().split(separator), function(text) {
        return { value: text };
      });
    } else {
      return [];
    }
  };

  var initialize_autocomplete = function(data_source, callback) {
    if (data_source.url) {
      $.get(data_source.url, function(data) {
        callback(data);
      });
    } else {
      callback(data_source);
    }
  };

  var get_data_source_from_element = function(element) {
    if (element.is("select")) {
      return map_options_to_items(element.children("option"));
    }
  };

  var map_options_to_items = function(options) {
    return $.map(options, function(option) {
      return {
        label: $(option).text(),
        value: $(option).val()
      }
    });
  };

  var create_input = function(element) {
    var name = extract_input_name(element);
    if (element.is("select")) {
      element = input_from_select(element);
    }

    return element.data("name", name)
                  .data("values", []);
  };

  var extract_input_name = function(element) {
    var name = element.attr("name");
    if (name.substring(name.length - 2) != "[]") {
      name = name + "[]";
    }
    element.removeAttr("name");
    return name;
  };

  var input_from_select = function(element) {
    return $("<input/>").attr({
      type: "text",
      id:   element.attr("id")
    }).replaceAll(element);
  };

})(jQuery);
